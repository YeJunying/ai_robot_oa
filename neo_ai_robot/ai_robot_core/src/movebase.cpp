#include "movebase.h"

#include <iomanip>
#include <cmath>
#include <iostream>

#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetMap.h>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "ai_robot_msgs/RecoveryStatus.h"
//#include "obstacle_avoid.h"
#include "ros/service.h"
// #include "simple_local_planner.h"

#include "globals.h"

namespace ai_robot {

MoveBase::MoveBase(tf2_ros::Buffer& tf)
    : tf_(tf),
      as_(nullptr),
      planner_plan_(nullptr),
      latest_plan_(nullptr),
      controller_plan_(nullptr),
      run_planner_(false),
      setup_(false),
      p_freq_change_(false),
      c_freq_change_(false),
      has_new_global_plan_(false),
      transform_tolerance_(0.5) {
  // Create an action server but do not start it automatically once initialized.
  as_ = new MoveBaseActionServer(ros::NodeHandle(), "ai_robot/movebase", [this](auto& goal){ executeCb(goal); }, false);

  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;

  recovery_trigger_ = kPlanningR;

  // std::string global_planner, local_planner;

  private_nh.param("planner_frequency", planner_freq_, 0.0);
  private_nh.param("controller_frequency", controller_freq_, 20.0);
  private_nh.param("planner_patience", planner_patience_, 5.0);
  private_nh.param("controller_patience", controller_patience_, 15.0);
  private_nh.param("max_planning_retries", max_planning_retries_, -1); // disabled by default
  private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
  private_nh.param("oscillation_distance",oscillation_distance_, 0.5);

  // Get map info
  mapinfo_srv_ = nh.serviceClient<nav_msgs::GetMap>("/static_map");
  getMapInfo();

  planner_ = std::make_shared<CloudPlanner>(mapinfo_.resolution);
  // tc_ = std::make_shared<SimpleLocalPlanner>(&mapinfo_);
  // tc_->initialize("simple_local_planner", &tf_);
//  tc_ = std::make_shared<ObstacleAvoid>( &mapinfo_);
  tc_ = std::make_shared<FalcoLocalPlanner>( &mapinfo_);
  tc_->initialize("obstacle_avoid", &tf_);

  // set up plan triple buffer
  planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
  latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
  controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

  // set up the planner's thread
  planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));

  // for commmanding the base
  vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0);

  ros::NodeHandle action_nh("movebase");
  action_goal_pub_ = action_nh.advertise<ai_robot_msgs::MoveBaseActionGoal>("goal", 1);
  recovery_status_pub_ = action_nh.advertise<ai_robot_msgs::RecoveryStatus>("recovery_status", 1);

  // for rviz to publish goal
  // ros::NodeHandle simple_nh("movebase_simple");
  // goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, [this](auto& goal){ goalCB(goal); });

  // makeplan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);

  // initial state
  state_ = kPlanning;

  recovery_index_ = 0;

  // start action server
  as_->start();
}

MoveBase::~MoveBase() {
  if (as_ != nullptr)
    delete as_;

  planner_thread_->interrupt();
  planner_thread_->join();

  delete planner_thread_;
  delete planner_plan_;
  delete latest_plan_;
  delete controller_plan_;

  // planner_.reset();
  // tc_.reset();
}

void MoveBase::getMapInfo() {
  nav_msgs::GetMap map;
  mapinfo_srv_.call(map);
  mapinfo_.resolution = map.response.map.info.resolution;
  mapinfo_.origin.x = map.response.map.info.origin.position.x;
  mapinfo_.origin.y = map.response.map.info.origin.position.y;
}

void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal) {
  ROS_DEBUG_NAMED("movebase", "In ROS goal callback, wrapping the PoseStamped in the action message and re-send to the server.");
  ai_robot_msgs::MoveBaseActionGoal action_goal;
  action_goal.header.stamp = ros::Time::now();
  action_goal.goal.target_pose = *goal;

  action_goal_pub_.publish(action_goal);
}

// bool MoveBase::planService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {
//   if (as_->isActive()) {
//     ROS_ERROR("move_base must be in an inactive state to make a plan for an external user");
//     return false;
//   }
//
//   std::vector<geometry_msgs::PoseStamped> global_plan;
//   if (!planner_->makePlan(start, req.goal, global_plan)) || global_plan.empty()) {
//     ROS_DEBUG_NAMED("movebase", "Failed to find a plan to exact goal of (%.2f, %.2f)", req.goal.pose.position.x, req.goal.pose.position.y);
//     return false
//   }
//
//   for (auto p : global_plan)
//     resp.plan.poses.push_back(p);
//
//   return true;
// }

void MoveBase::transformPositionFromMercatorToMap(double in_x, double in_y, double& out_x, double& out_y) {
  bluetooth::Transform::merca2plane(in_x, in_y, out_x, out_y, mapinfo_.resolution);
  ROS_INFO("origin_x: %f, origin_y: %f", mapinfo_.origin.x, mapinfo_.origin.y);
  out_x += mapinfo_.origin.x;
  out_y += mapinfo_.origin.y;
}

bool MoveBase::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
  plan.clear();

  geometry_msgs::PoseStamped global_pose;
  if (!getRobotPose(global_pose)) {
    ROS_WARN("Unable to get starting pose of robot, unable to make global plan");
    return false;
  }

  geometry_msgs::PoseStamped& start = global_pose;
  start.pose.position.x -= mapinfo_.origin.x;
  start.pose.position.y -= mapinfo_.origin.y;

  if (!planner_->makePlan(start, goal, plan) || plan.empty()) {
    ROS_DEBUG_NAMED("movebase", "Failed to find a plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
    return false;
  }

  // for (auto& waypoint : plan) {
    // transformPositionFromMercatorToMap(waypoint.pose.position.x,
    //                                    waypoint.pose.position.y,
    //                                    waypoint.pose.position.x,
    //                                    waypoint.pose.position.y);
  // }
  return true;
}

void MoveBase::publishVelocity(const geometry_msgs::Twist& cmd_vel) {
  if (tc_->isLockedByMan()) {
    return;
  }
  vel_pub_.publish(cmd_vel);
}

void MoveBase::publishZeroVelocity() {
  if (g::in_man_mode)
    return;
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  vel_pub_.publish(cmd_vel);
}

geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg) {
  // geometry_msgs::PoseStamped global_pose;
  // global_pose.header.stamp = ros::Time();
  // return global_pose;
  return goal_pose_msg;
};

void MoveBase::wakePlanner(const ros::TimerEvent& event) {
  planner_cond_.notify_one();
}

void MoveBase::planThread() {
  ROS_DEBUG_NAMED("movebase_plan_thread", "Starting planner thread...");
  ros::NodeHandle n;
  ros::Timer timer;
  bool wait_for_wake = false;
  boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);

  while (n.ok()) {
    while (wait_for_wake || !run_planner_) {
      planner_cond_.wait(lock);
      wait_for_wake = false;
    }
    ros::Time start_time = ros::Time::now();

    // Time to plan.
    geometry_msgs::PoseStamped temp_goal = planner_goal_;
    lock.unlock();
    ROS_DEBUG_NAMED("movebase_plan_thread", "Planning...");

    // Run planner.
    planner_plan_->clear();
    bool got_plan = n.ok() && makePlan(temp_goal, *planner_plan_);

    if (got_plan) {
      ROS_DEBUG_NAMED("movebase_plan_thread", "Got plan with %zu points", planner_plan_->size());
      std::vector<geometry_msgs::PoseStamped> *temp_plan = planner_plan_;

      lock.lock();
      planner_plan_ = latest_plan_;
      latest_plan_ = temp_plan;
      t_last_valid_plan_ = ros::Time::now();
      planning_retries_ = 0;
      has_new_global_plan_ = true;

      ROS_DEBUG_NAMED("movebase_plan_thread", "Generated a plan from the cloud_global_planner");

      // print plan for test
      for (const auto p : *latest_plan_)
        std::cout << ">>>>>> global waypoint: x=" << std::fixed << std::setprecision(10) << p.pose.position.x << ", y=" << p.pose.position.y << '\n';

      // Make sure we only start the controller if we still haven't reached the goal
      if (run_planner_)
        state_ = kControlling;
      if (planner_freq_ <= 0)
        run_planner_ = false;
      lock.unlock();
    } else if (state_ = kPlanning) { // failed to get a plan but still in planning stage (robit isn't moving)
      ROS_DEBUG_NAMED("movebase_plan_thread", "No plan...");
      ros::Time t_attempt_end = t_last_valid_plan_ + ros::Duration(planner_patience_);
      lock.lock();
      ++planning_retries_;
      if (run_planner_ && (ros::Time::now() > t_attempt_end || planning_retries_ > uint32_t(max_planning_retries_))) {
        state_ = kClearing;
        run_planner_ = false;
        publishZeroVelocity();
        recovery_trigger_ = kPlanningR;
      }
      lock.unlock();
    }

    // Take the mutex for next iteration
    lock.lock();

    // Set up sleep interface is needed
    if (planner_freq_ > 0) {
      ros::Duration sleep_time = (start_time + ros::Duration(1.0 / planner_freq_)) - ros::Time::now();
      if (sleep_time > ros::Duration(0.0)) {
        wait_for_wake = true;
        timer = n.createTimer(sleep_time, &MoveBase::wakePlanner, this);
      }
    }
  }
}

void MoveBase::executeCb(const ai_robot_msgs::MoveBaseGoalConstPtr& move_base_goal) {
  geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);

  publishZeroVelocity();

  // Wake global planner
  boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
  planner_goal_ = goal;
  run_planner_ = true;
  planner_cond_.notify_one();
  lock.unlock();

  current_goal_pub_.publish(goal);

  ros::Rate r(controller_freq_);

  t_last_valid_control_ = ros::Time::now();
  t_last_valid_plan_ = ros::Time::now();
  t_last_oscillation_reset_ = ros::Time::now();
  planning_retries_ = 0;

  ros::NodeHandle n;
  while (n.ok()) {
    if (as_->isPreemptRequested()) {
      if (as_->isNewGoalAvailable()) { // new goal received
        ai_robot_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();
        
        goal = goalToGlobalFrame(new_goal.target_pose);

        state_ = kPlanning;

        lock.lock();
        planner_goal_ = goal;
        run_planner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        current_goal_pub_.publish(goal);

        t_last_valid_control_ = ros::Time::now();
        t_last_valid_plan_ = ros::Time::now();
        t_last_oscillation_reset_ = ros::Time::now();
        planning_retries_ = 0;
      } else { // cancelled
        resetState();
        ROS_DEBUG_NAMED("movebase", "Move base preempting the current goal");
        as_->setPreempted();
        return;
      }
    }

    ros::WallTime t_start = ros::WallTime::now();

    bool done = executeCycle(goal);

    if (done)
      return;

    ros::WallDuration t_diff = ros::WallTime::now() - t_start;
    ROS_DEBUG_NAMED("movebase", "Full control cycle time: %.9f\n", t_diff.toSec());

    r.sleep();
    if (r.cycleTime() > ros::Duration(1 / controller_freq_) && state_ == kControlling)
      ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_freq_, r.cycleTime().toSec());
  }

  // Wake up the planner thread so that it can exit cleanly
  lock.lock();
  run_planner_ = true;
  planner_cond_.notify_one();
  lock.unlock();

  // The action node gets killed
  as_->setAborted(ai_robot_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
  return;
}

// NOTE: For test now, need to use bluetooth position.
bool MoveBase::getRobotPose(geometry_msgs::PoseStamped& global_pose) {
  tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
  geometry_msgs::PoseStamped robot_pose;
  tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
  robot_pose.header.frame_id = "base_link";
  robot_pose.header.stamp = ros::Time();
  ros::Time curr_time = ros::Time::now();

  try {
    tf_.transform(robot_pose, global_pose, "map");
    // global_pose.pose.position.x -= mapinfo_.origin.x;
    // global_pose.pose.position.y -= mapinfo_.origin.y;
    ROS_INFO("start pose in map: %.2f, %.2f", global_pose.pose.position.x, global_pose.pose.position.y);
  } catch (tf2::LookupException& ex) {
    ROS_ERROR_THROTTLE(1.0, "No Transform available Error lookup up robot pose: %s\n", ex.what());
    return false;
  } catch (tf2::ConnectivityException& ex) {
    ROS_ERROR_THROTTLE(1.0, "Connectivity Error lookup up robot pose: %s\n", ex.what());
    return false;
  } catch (tf2::ExtrapolationException& ex) {
    ROS_ERROR_THROTTLE(1.0, "Extrapolation Error lookup up robot pose: %s\n", ex.what());
    return false;
  }

  if (!global_pose.header.stamp.isZero() && curr_time.toSec() - global_pose.header.stamp.toSec() > transform_tolerance_) {
    ROS_WARN_THROTTLE(1.0, "Transform from %s to %s timeout. Current time: %.4f, pose stamp: %.4f, tolerance: %.4f",
                      "base_link", "map", curr_time.toSec(), global_pose.header.stamp.toSec(), transform_tolerance_);
    return false;
  }

  // NOTE: manual test
  // global_pose.header.stamp = ros::Time::now();
  // global_pose.pose.position.x = 13519960.290347446;
  // global_pose.pose.position.y = 3614355.438641976;

  return true;
}

double MoveBase::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2) {
  return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
}

bool MoveBase::executeCycle(geometry_msgs::PoseStamped& goal) {
  geometry_msgs::Twist cmd_vel;
  // geometry_msgs::PoseStamped global_pose;
  // getRobotPose(global_pose);
  // const geometry_msgs::PoseStamped& cur_pos = global_pose;

  // ai_robot_msgs::MoveBaseFeedback feedback;
  // feedback.base_position = cur_pos;
  // as_->publishFeedback(feedback);

  // if (distance(cur_pos, oscillation_pose_) >= oscillation_distance_) {
  //   t_last_oscillation_reset_ = ros::Time::now();
  //   oscillation_pose_ = cur_pos;
  //
  //   if (recovery_trigger_ == kOscillationR)
  //     recovery_index_ = 0;
  // }

  if (has_new_global_plan_) {
    has_new_global_plan_ = false;

    ROS_DEBUG_NAMED("movebase", "Got a new plan...swap pointers");

    std::vector<geometry_msgs::PoseStamped> *temp_plan = controller_plan_;

    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    controller_plan_ = latest_plan_;
    latest_plan_ = temp_plan;
    lock.unlock();
    ROS_DEBUG_NAMED("movebase", "pointers swapped");

    if (!tc_->setPlan(*controller_plan_)) {
      // Abort and shutdown
      ROS_ERROR("Failed to pass global plan to the controller, aborting.");
      resetState();

      lock.lock();
      run_planner_ = false;
      lock.unlock();

      as_->setAborted(ai_robot_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
      return true;
    }

    if (recovery_trigger_ == kPlanningR)
      recovery_index_ = 0;
  }

  switch (state_) {
    case kPlanning:
      {
        boost::recursive_mutex::scoped_lock lock(planner_mutex_);
        run_planner_ = true;
        planner_cond_.notify_one();
      }
      ROS_DEBUG_NAMED("movebase", "Waiting for plan, in the planning state.");
      break;
    case kControlling:
    {
      ROS_DEBUG_NAMED("movebase", "In controlling state.");
      if (tc_->isGoalReached()) {
        ROS_DEBUG_NAMED("movebase", "Goal reached!");
        resetState();
        // Disable the planner thread
        boost::recursive_mutex::scoped_lock lock(planner_mutex_);
        run_planner_ = false;
        lock.unlock();

        as_->setSucceeded(ai_robot_msgs::MoveBaseResult(), "Goal reached.");
        return true;
      }

      // if (oscillation_timeout_ > 0.0 && t_last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now()) {
      //   publishZeroVelocity();
      //   state_ = kClearing;
      //   recovery_trigger_ = kOscillationR;
      // }
      
      // tc_->safeZoneView();
      std::lock_guard<std::mutex> oa_mtx(g::oa_lock);
      if (tc_->computeVelocityCommands(cmd_vel)) {
        ROS_INFO_NAMED("movebase", "Got a valid command from the local planner: vx=%.2f, vy=%.2f, rvz=%.2f", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
        t_last_valid_control_ = ros::Time::now();
        // vel_pub_.publish(cmd_vel);
        publishVelocity(cmd_vel);
        if (recovery_trigger_ == kControllingR)
          recovery_index_ = 0;
      } else {
        ROS_DEBUG_NAMED("movebase", "The local planner could not find a valid plan");
        ros::Time t_attempt_end = t_last_valid_control_ + ros::Duration(controller_patience_);

        if (ros::Time::now() > t_attempt_end) {
          publishZeroVelocity();
          state_ = kClearing;
          recovery_trigger_ = kControllingR;
        } else {
          t_last_valid_plan_ = ros::Time::now();
          planning_retries_ = 0;
          state_ = kPlanning;
          publishZeroVelocity();

          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
          run_planner_ = true;
          planner_cond_.notify_one();
          lock.unlock();
        }
      }
      break;
    }
    case kClearing:
    {
      ROS_DEBUG_NAMED("movebase", "In clearing/recovery state");
      // NOTE: not implemented yet
      if (0) {
      } else {
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        run_planner_ = false;
        lock.unlock();
        ROS_DEBUG_NAMED("movebase_recovery", "Something should abort after this");
        as_->setAborted(ai_robot_msgs::MoveBaseResult(), "Recovery mechanism not implemented yet");
        resetState();
        return true;
      }
      break;
    }
    default:
    {
      ROS_ERROR("This case should never be hit, something is wrong, aboring");
      resetState();
      boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
      run_planner_ = false;
      lock.unlock();
      as_->setAborted(ai_robot_msgs::MoveBaseResult(), "Reached a case that should not be hit in movebase. This is a bug!");
      return true;
    }
  }

  return false;
}

void MoveBase::resetState() {
  boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
  run_planner_ = false;
  lock.unlock();

  state_ = kPlanning;
  recovery_index_ = 0;
  recovery_trigger_ = kPlanningR;
  publishZeroVelocity();
}

}
