#include "simple_local_planner.h"

#include <cmath>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/GetMap.h>
#include <ros/node_handle.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>

#include "ai_robot_msgs/OAVelocity.h"
#include "ai_robot_msgs/SetRobotPose.h"
#include "ai_robot_msgs/SetTargetPosition.h"

#include "bluetooth.h"
#include "geometry_msgs/PoseStamped.h"
#include "globals.h"
#include "man_control.h"
#include "std_msgs/Empty.h"
#include "util.h"

namespace ai_robot {

SimpleLocalPlanner::SimpleLocalPlanner(MapInfo *mapinfo)
    : initialized_(false),
      from_index_(0),
      to_index_(0),
      locked_by_man_(false),
      imu_has_mag_(false),
      has_new_plan_(false),
      goal_reached_(false),
      waypoint_reached_(false),
      turn_done_(false),
      new_hdg_updated_(false),
      tf_(nullptr),
      hdg_(0.0),
      blue_dist_to_waypoint_(99),
      mapinfo_(mapinfo),
      next_dist_from_odom_(0),
      oa_vel_vx_(0.0),
      oa_vel_rz_(0.0),
      transform_tolerance_(0.5) {
}

SimpleLocalPlanner::~SimpleLocalPlanner() {
}

// Eigen::Vector2d SimpleLocalPlanner::getPositionFromOdomToMercator(const Eigen::Vector2d& pos) {
//   return rotm_odom2merca_ * pos + tr_odom2merca_;
// }

// Eigen::Vector2d SimpleLocalPlanner::getPositionFromMercatorToOdom(const Eigen::Vector2d& pos) {
//   return rotm_odom2merca_.transpose() * (pos - tr_odom2merca_);
// }

// void SimpleLocalPlanner::transformPositionFromOdomToMercator(const Eigen::Vector2d& src, Eigen::Vector2d& tgt) {
//   tgt = rotm_odom2merca_ * src + tr_odom2merca_;
// }

// void SimpleLocalPlanner::transformPositionFromMercatorToOdom(const Eigen::Vector2d& src, Eigen::Vector2d& tgt) {
//   tgt = rotm_odom2merca_.transpose() * (src - tr_odom2merca_);
// }

void SimpleLocalPlanner::transformPositionFromMercatorToMap(const Eigen::Vector2d& in, Eigen::Vector2d& out) {
  bluetooth::Transform::merca2plane(in(0), in(1), out(0), out(1), mapinfo_->resolution);
  out[0] += mapinfo_->origin.x;
  out[1] += mapinfo_->origin.y;
}

void SimpleLocalPlanner::transformPositionFromMercatorToMap(double in_x, double in_y, double& out_x, double& out_y) {
  bluetooth::Transform::merca2plane(in_x, in_y, out_x, out_y, mapinfo_->resolution);
  out_x += mapinfo_->origin.x;
  out_y += mapinfo_->origin.y;
}

// void SimpleLocalPlanner::transformPositionFromMapToMercator(const Eigen::Vector2d& in, Eigen::Vector2d& out) {
//   bluetooth::Transform::plane2merca(in(0), in(1), out(0), out(1));
// }

double SimpleLocalPlanner::transformHeadingFromNeuToMap(double hdg) {
  return util::normAngle(hdg - defl_from_neu_to_map_);
}

double SimpleLocalPlanner::transformHeadingFromMapToNeu(double hdg) {
  return util::normAngle(hdg + defl_from_neu_to_map_);
}

double SimpleLocalPlanner::transformHeadingFromNwuToMap(double hdg) {
  return util::normAngle(hdg - defl_from_nwu_to_map_);
}

double SimpleLocalPlanner::transformHeadingFromMapToNwu(double hdg) {
  return util::normAngle(hdg + defl_from_nwu_to_map_);
}

double SimpleLocalPlanner::transformHeadingFromNwuToNeu(double hdg) {
  return util::normAngle(hdg + M_PI_2);
}

double SimpleLocalPlanner::transformHeadingFromNeuToNwu(double hdg) {
  return util::normAngle(hdg - M_PI_2);
}

double SimpleLocalPlanner::getDistanceXY(const geometry_msgs::Pose& p1, const geometry_msgs::PoseStamped& p2) {
  return std::hypot(p1.position.x - p2.pose.position.x, p1.position.y - p2.pose.position.y);
}

double SimpleLocalPlanner::getDistanceXY(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2) {
  return std::hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
}

double SimpleLocalPlanner::getDistanceXY(double x1, double y1, double x2, double y2) {
  return std::hypot(x2 - x1, y2 - y1);
}

// double SimpleLocalPlanner::getAngleDifference(double new_hdg, double cur_hdg) {
//   double yaw = util::getAngleDiff(new_hdg, cur_hdg); 
//   if (is_merca_aligned_to_map_) {
//     yaw += defl_from_neu_to_map_;
//     // if (yaw > M_PI)
//     //   yaw -= M_PI * 2;
//     // else if (yaw <= -M_PI)
//     //   yaw += M_PI * 2;
//   }
//   // Ensure yaw within (-180, 180]
//   return util::normAngle(yaw);
// }

// Working directly with mercator
double SimpleLocalPlanner::getAngleDifference(double new_hdg, double cur_hdg) {
  double yaw = util::getAngleDiff(new_hdg, cur_hdg); 
  // if (is_merca_aligned_to_map_) {
    // yaw += defl_from_neu_to_map_;
    // if (yaw > M_PI)
    //   yaw -= M_PI * 2;
    // else if (yaw <= -M_PI)
    //   yaw += M_PI * 2;
  // }
  // Ensure yaw within (-180, 180]
  return util::normAngle(yaw);
}

void SimpleLocalPlanner::initialize(std::string name, tf2_ros::Buffer *tf) {
 if (!isInitialized()) {
    ros::NodeHandle private_nh("~/" + name);
    ros::NodeHandle nh;
    // imu_sub_ = nh.subscribe("/imu", 1, &SimpleLocalPlanner::imuCB, this);
    // odom_sub_ = nh.subscribe("/odom", 1, &SimpleLocalPlanner::odomCB, this);
    oa_vel_sub_ = nh.subscribe("/oa_vel", 1, &SimpleLocalPlanner::velocityCB, this);
    manctl_state_sub_ = nh.subscribe("ai_robot/manctl_state", 1, &SimpleLocalPlanner::manctlStateCB, this);
    oa_pose_srv_ = nh.serviceClient<ai_robot_msgs::SetRobotPose>("/oa_robot_pose");
    oa_tgt_srv_ = nh.serviceClient<ai_robot_msgs::SetTargetPosition>("/oa_target_pos");
    imu_sub_ = nh.subscribe("/imu", 1, &SimpleLocalPlanner::imuCB, this);
    odom_sub_ = nh.subscribe("/odom", 1, &SimpleLocalPlanner::odomCB, this);
    blue_sub_ = nh.subscribe("/blue_pose", 1, &SimpleLocalPlanner::blueCB, this);
    manctl_state_sub_ = nh.subscribe("ai_robot/manctl_state", 1, &SimpleLocalPlanner::manctlStateCB, this);
    goal_reached_ = false;
    waypoint_reached_ = false;
    tf_ = tf;

    private_nh.param("defl_from_neu_to_map", defl_from_neu_to_map_, 19.0);
    private_nh.param("defl_from_neu_to_map", defl_from_nwu_to_map_, -71.0);
    private_nh.param("is_mercator_aligned_to_map", is_merca_aligned_to_map_, false);
    private_nh.param("max_vel_x", max_vel_x_, 0.5);
    private_nh.param("max_vel_y", max_vel_y_, 0.25);
    private_nh.param("max_vel_theta", max_vel_theta_, 1.87);
    private_nh.param("xy_goal_tolerance_blue", xy_goal_tolerance_blue_, 2.5);
    private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 1.0);
    private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
    private_nh.param("robot_width", robot_width_, 0.15);
    private_nh.param("robot_safe_gap", robot_safe_gap_, 0.2);
    private_nh.param("imu_hdg_offset_to_map", imu_hdg_offset_to_map_, M_PI_2 + 19.0);
    private_nh.getParam("imu_has_mag", imu_has_mag_);
    // private_nh.param("imu_hdg_offset_to_map", imu_hdg_offset_to_map_, M_PI_2 + 19.0);
    defl_from_neu_to_map_ = util::deg2rad(defl_from_neu_to_map_);
    defl_from_nwu_to_map_ = util::deg2rad(defl_from_nwu_to_map_);

    // mapinfo_srv_ = nh.serviceClient<nav_msgs::GetMap>("/static_map");
    // getMapInfo();
  }
}

bool SimpleLocalPlanner::getRobotPose(geometry_msgs::PoseStamped& global_pose) {
  tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
  geometry_msgs::PoseStamped robot_pose;
  tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
  robot_pose.header.frame_id = "base_link";
  robot_pose.header.stamp = ros::Time();
  ros::Time curr_time = ros::Time::now();

  try {
    tf_->transform(robot_pose, global_pose, "map");
    // global_pose.pose.position.x -= mapinfo_->origin.x;
    // global_pose.pose.position.y -= mapinfo_->origin.y;
    // ROS_INFO("start pose in map: %.2f, %.2f", global_pose.pose.position.x, global_pose.pose.position.y);
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

  return true;
}

// void SimpleLocalPlanner::initialPoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
//   pos_(0) = msg->pose.pose.position.x;
//   pos_(1) = msg->pose.pose.position.y;
// }

// void SimpleLocalPlanner::imuCB(const sensor_msgs::ImuConstPtr& msg) {
//   double roll, pitch, yaw;
//   util::quat2eul(roll, pitch, yaw, msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
//   // Transform from NEU frame to map frame
//   hdg_ = transformHeadingFromNwuToMap(yaw);
// }

void SimpleLocalPlanner::imuCB(const sensor_msgs::ImuConstPtr& msg) {
  double roll, pitch, yaw;
  util::quat2eul(roll, pitch, yaw, msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  // Transform from NEU frame to map frame
  hdg_ = imu_has_mag_ ? transformHeadingFromNwuToMap(yaw) : yaw;
}

// To work directly with mercator frame
// void SimpleLocalPlanner::imuCB(const sensor_msgs::ImuConstPtr& msg) {
  // double roll, pitch, yaw;
  // util::quat2eul(roll, pitch, yaw, msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  // Transform from NEU frame to map frame
  // hdg_ = transformHeadingFromNwuToNeu(yaw);
  // hdg_ = yaw;
// }

// void SimpleLocalPlanner::odomCB(const nav_msgs::OdometryConstPtr& msg) {
//   double r, p, y;
//   // tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
//   // tf2::Matrix3x3 m(q);
//   // m.getRPY(r, p, y);
//   // hdg_ = y;
//
//   pos_[0] = msg->pose.pose.position.x;
//   pos_[1] = msg->pose.pose.position.y;
//
//   odom_pose_.header.stamp = msg->header.stamp;
//   odom_pose_.header.frame_id = msg->header.frame_id;
//   odom_pose_.pose = msg->pose.pose;
//
//   if (!has_new_plan_)
//     return;
//
//   // Currently we assume the odom frame has no drift and the origin of the map
//   // frame is at the same position as the odom frame.
//   // odom_dist_to_waypoint_ = getDistanceXY(msg->pose.pose, global_plan_odom_[to_index_]);
//
//   geometry_msgs::PoseStamped map_to_base;
//   odom_pose_.header.stamp = msg->header.stamp;
//   odom_pose_.header.frame_id = msg->header.frame_id;
//   odom_pose_.pose = msg->pose.pose;
//   try {
//     tf_->transform(odom_pose_, map_to_base, "map", ros::Duration(0.1));
//     // ROS_INFO("**** robot position in map: %.2f, %.2f",
//     //          map_to_base.pose.position.x,
//     //          map_to_base.pose.position.y);
//     odom_dist_to_waypoint_ = getDistanceXY(map_to_base.pose.position.x,
//                                            map_to_base.pose.position.y,
//                                            global_plan_[to_index_].pose.position.x,
//                                            global_plan_[to_index_].pose.position.y);
//     // tf2::Quaternion q;
//     // tf2::fromMsg(map_to_base.pose.orientation, q);
//     // hdg_ = tf2::getYaw(q);
//   } catch (const tf2::TransformException& ex) {
//     ROS_DEBUG("Failed to transform from odom frame to map frame: %s", ex.what());
//     return;
//   }
// }

// Working directly with mercator
void SimpleLocalPlanner::odomCB(const nav_msgs::OdometryConstPtr& msg) {
  double r, p, y;
  // tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  // tf2::Matrix3x3 m(q);
  // m.getRPY(r, p, y);
  // hdg_ = y;

  pos_[0] = msg->pose.pose.position.x;
  pos_[1] = msg->pose.pose.position.y;

  // odom_pose_.header.stamp = msg->header.stamp;
  // odom_pose_.header.frame_id = msg->header.frame_id;
  // odom_pose_.pose = msg->pose.pose;
  //
  // if (!has_new_plan_)
  //   return;
  //
  // // Currently we assume the odom frame has no drift and the origin of the map
  // // frame is at the same position as the odom frame.
  // // odom_dist_to_waypoint_ = getDistanceXY(msg->pose.pose, global_plan_odom_[to_index_]);
  //
  // geometry_msgs::PoseStamped map_to_base;
  // odom_pose_.header.stamp = msg->header.stamp;
  // odom_pose_.header.frame_id = msg->header.frame_id;
  // odom_pose_.pose = msg->pose.pose;
  // try {
  //   tf_->transform(odom_pose_, map_to_base, "map", ros::Duration(0.1));
  //   // ROS_INFO("**** robot position in map: %.2f, %.2f",
  //   //          map_to_base.pose.position.x,
  //   //          map_to_base.pose.position.y);
  //   odom_dist_to_waypoint_ = getDistanceXY(map_to_base.pose.position.x,
  //                                          map_to_base.pose.position.y,
  //                                          global_plan_[to_index_].pose.position.x,
  //                                          global_plan_[to_index_].pose.position.y);
  //   // tf2::Quaternion q;
  //   // tf2::fromMsg(map_to_base.pose.orientation, q);
  //   // hdg_ = tf2::getYaw(q);
  // } catch (const tf2::TransformException& ex) {
  //   ROS_DEBUG("Failed to transform from oom frame to map frame: %s", ex.what());
  //   return;
  // }

  // odom_dist_to_waypoint_ = getDistanceXY(pos_[0], pos_[1], waypoint_in_map_[0], waypoint_in_map_[1]);
  double dist = getDistanceXY(pos_[0], pos_[1], 0, 0);
  odom_dist_to_waypoint_ = fabs(dist - next_dist_from_odom_);
}

// void SimpleLocalPlanner::blueCB(const geometry_msgs::PointStampedConstPtr& msg) {
//   // ROS_INFO("Bluetooth position: %.2f, %.2f", msg->point.x, msg->point.y);
//   // Only within 3 meters to the next waypoint, consider bluetooth.
//   blue_pos_[0] = msg->point.x;
//   blue_pos_[1] = msg->point.y;
//
//   if (!has_new_plan_)
//     return;
//
//   double x, y;
//   // Mercator frame to map frame
//   transformPositionFromMercatorToMap(msg->point.x, msg->point.y, x, y);
//   blue_dist_to_waypoint_ = getDistanceXY(x, y, global_plan_[to_index_].pose.position.x, global_plan_[to_index_].pose.position.y);
// }

// Directly working with mercator frame
void SimpleLocalPlanner::blueCB(const geometry_msgs::PointStampedConstPtr& msg) {
  // ROS_INFO("Bluetooth position: %.2f, %.2f", msg->point.x, msg->point.y);
  // Only within 3 meters to the next waypoint, consider bluetooth.
  blue_pos_[0] = msg->point.x;
  blue_pos_[1] = msg->point.y;

  if (!has_new_plan_)
    return;

  // double x, y;
  // Mercator frame to map frame
  // transformPositionFromMercatorToMap(msg->point.x, msg->point.y, x, y);
  blue_dist_to_waypoint_ = getDistanceXY(blue_pos_[0], blue_pos_[1], global_plan_[to_index_].pose.position.x, global_plan_[to_index_].pose.position.y);
}

void SimpleLocalPlanner::velocityCB(const ai_robot_msgs::OAVelocityConstPtr& msg) {
  oa_vel_vx_ = msg->vx / 100; // cm -> m
  oa_vel_rz_ = util::deg2rad(msg->rz);  // degrees to radians
}

void SimpleLocalPlanner::manctlStateCB(const std_msgs::UInt8ConstPtr& msg) {
  switch (static_cast<ManControlState>(msg->data)) {
    case ManControlState::kLocked:
      ROS_INFO_NAMED("local_planner", "Locked by man");
      locked_by_man_ = true;
      break;
    case ManControlState::kUnlocked:
      ROS_INFO_NAMED("local_planner", "Unlocked by man");
      locked_by_man_ = false;
      break;
    default:
      break;
  }
}

// void OdomAndBluePositionCallback(const nav_msgs::OdometryConstPtr& odom_msg, const geometry_msgs::PointStampedConstPtr& blue_msg) {
//   double blue_x, blue_y;
//   double blue_dist, odom_dist;
//   // bluetooth::Transform::merca2plane(blue_msg->point.x, blue_msg->point.y, blue_x, blue_y);
//   transformPositionFromMercatorToMap(blue_msg->point.x, blue_msg->point.y, blue_x, blue_y);
//   blue_dist = getDistanceXY(blue_x, blue_y, global_plan_[to_index_].pose.position.x, global_plan_[to_index_].pose.position.y);
//
//   geometry_msgs::PoseStamped odom_pose, map_to_base;
//   odom_pose.header.stamp = odom_msg->header.stamp;
//   odom_pose.header.frame_id = odom_msg->header.frame_id;
//   odom_pose.pose = odom_msg->pose.pose;
//   try {
//     tf_->transform(odom_pose, map_to_base, "map", ros::Duration(0.1))
//     odom_dist = getDistanceXY(map_to_base.pose.position.x, map_to_base.pose.position.y,
//                           global_plan_[to_index_].pose.position.x, global_plan_[to_index_].pose.position.y);
//   } catch (const tf2::TransformException& ex) {
//     ROS_DEBUG("Failed to transform from odom frame to map frame: %s", ex.what());
//     return;
//   }
//   
//   if (blue_dist <= dist_thresh_blue_ && odom_dist <= dist_thresh_odom_)
//     waypoint_reached_ = true;
// }

void SimpleLocalPlanner::updateHeading(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2) {
  // util::enuHdg(new_hdg_, p1.pose.position.x, p1.pose.position.y, p2.pose.position.x, p2.pose.position.y);
  // std::cout << ">>>> Next heading: " << util::rad2deg(new_hdg_) << '\n';
  new_hdg_ = std::atan2(p2.pose.position.y - p1.pose.position.y, p2.pose.position.x - p1.pose.position.x);
}

// bool SimpleLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
//   global_plan_ = orig_global_plan;
//
//   for (auto& p : global_plan_) {
//     p.pose.position.x += mapinfo_->origin.x;
//     p.pose.position.y += mapinfo_->origin.y;
//     std::cout << ">>>>>> waypoint in map frame: x=" << std::fixed << std::setprecision(10) << p.pose.position.x << ", y=" << p.pose.position.y << '\n';
//   }
//
//   if (global_plan_.size() < 2) // has start point only
//     return true;
//
//   // NOTE: just for test!
//   // hdg_ = M_PI;
//
//   has_new_plan_ = true;
//
//   return true;
// }

// Working directly with mercator frame
bool SimpleLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
  global_plan_ = orig_global_plan;

  // if (global_plan_.size() < 2) // has start point only
  //   return true;

  // NOTE: just for test!
  // hdg_ = M_PI;
  pos_[0] = global_plan_.front().pose.position.x;
  pos_[1] = global_plan_.front().pose.position.y;

  has_new_plan_ = true;

  geometry_msgs::PoseStamped initial_pose;
  getRobotPose(initial_pose); // robot pose in map frame
  double roll, pitch, yaw;
  util::quat2eul(roll, pitch, yaw,
                 initial_pose.pose.orientation.x,
                 initial_pose.pose.orientation.y,
                 initial_pose.pose.orientation.z,
                 initial_pose.pose.orientation.w);
  double waypoint_orient = std::atan2(global_plan_[0].pose.position.y - initial_pose.pose.position.y,
                                      global_plan_[0].pose.position.x - initial_pose.pose.position.x);

  ai_robot_msgs::SetRobotPose robot_pose_msg;
  robot_pose_msg.request.x = initial_pose.pose.position.x;
  robot_pose_msg.request.y = initial_pose.pose.position.y;
  robot_pose_msg.request.theta = yaw;
  // msg.request.guide_angle = util::rad2deg(util::getAngleDiff(waypoint_orient, yaw));
  oa_pose_srv_.waitForExistence();
  oa_pose_srv_.call(robot_pose_msg);

  ai_robot_msgs::SetTargetPosition tgt_pos_msg;
  tgt_pos_msg.request.x = orig_global_plan[0].pose.position.x;
  tgt_pos_msg.request.y = orig_global_plan[0].pose.position.y;
  oa_tgt_srv_.waitForExistence();
  oa_tgt_srv_.call(tgt_pos_msg);

  goal_reached_ = false;

  return true;
}

bool SimpleLocalPlanner::computeVelocityCommands1(geometry_msgs::Twist& cmd_vel) {
  // static ros::Time t = ros::Time::now();
  static double rz = 0;
  if (locked_by_man_)
    return true;

  if (goal_reached_)
    goal_reached_ = false;

  // const auto& waypoint = global_plan_odom_[to_index_];
  // double dist = getDistanceXY(pos_(0), pos_(1), waypoint.pose.position.x, waypoint.pose.position.y);
  if (!waypoint_reached_ && (blue_dist_to_waypoint_ < xy_goal_tolerance_blue_ || odom_dist_to_waypoint_ < xy_goal_tolerance_)) {
    waypoint_reached_ = true;
    if (to_index_ == global_plan_.size() - 1) {
      goal_reached_ = true;
      return true;
    } else {
      from_index_ = to_index_++;
      next_dist_from_odom_ =
        getDistanceXY(global_plan_[to_index_].pose.position.x,
                      global_plan_[to_index_].pose.position.y,
                      global_plan_[0].pose.position.x,
                      global_plan_[0].pose.position.y);
      turn_done_ = false;
      blue_dist_to_waypoint_ = 99;
      std::cout << "from waypoint: " << from_index_ << " to waypoint: " << to_index_ << '\n';
    }
  }

  // if (dist < 0.1 && !waypoint_reached_) { // next waypoint reached
  //   waypoint_reached_ = true;
  //   if (to_index_ == global_plan_.size() - 1) {
  //     goal_reached_ = true;
  //     return true;
  //   }  else {
  //     from_index_ = to_index_++;
  //     turn_done_ = false;
  //     std::cout << "from waypoint: " << from_index_ << " to waypoint: " << to_index_ << '\n';
  //   }
  // }

  std::cout << ">>>> Odom distance to waypoint " << to_index_ << ": " << odom_dist_to_waypoint_ << '\n';
  if (waypoint_reached_) { // Change heading.
    if (!turn_done_ && !new_hdg_updated_) {
      // updateHeading(global_plan_odom_[from_index_], global_plan_odom_[to_index_]); // update new_hdg_
      updateHeading(global_plan_[from_index_], global_plan_[to_index_]); // update new_hdg_
      new_hdg_updated_ = true;
    }
    // NOTE:: just for test, need to be removed as hdg is updated by odom callback.
    // double dt = (ros::Time::now() - t).toSec();
    // hdg_ += rz * dt; // NOTE:: test only
    // std::cout << "rz: " << rz << " dt: " << dt << " hdg: " << hdg_ << " new hdg: " << new_hdg_ << '\n';
    double yaw = getAngleDifference(new_hdg_, hdg_);
    std::cout << "cur hdg: " << hdg_ << " new hdg: " << new_hdg_ << " yaw: " << yaw << '\n';
    if (std::fabs(yaw) > 0.3) {
      cmd_vel.angular.z = 0.3 * std::fabs(yaw) / yaw;
    } else {
      cmd_vel.angular.z = yaw;
      if (std::fabs(yaw) <= yaw_goal_tolerance_) {
        std::cout << "----> turn finished\n";
        turn_done_ = true;
        new_hdg_updated_ = false;
        waypoint_reached_ = false;
      }
    }
    // rz = cmd_vel.angular.z; // NOTE:: test only
  } 

  // Currently, we stops the robot at the corner and switches it to spin mode to perform a turn.
  // The distance to the waypoint is simply computed upon the odometric pose which is inaccurate for long run.
  // We need a better algorithm to detect corners and make robot turn more smoothly.
  else if (odom_dist_to_waypoint_ < 2) {
    cmd_vel.linear.x = odom_dist_to_waypoint_ / 4;
  } else {
    cmd_vel.linear.x = max_vel_x_;
  }

  // t = ros::Time::now();
  return true;
}

bool SimpleLocalPlanner::isGoalReached() {
  static geometry_msgs::PoseStamped cur_pose;
  static double dist;

  if (!goal_reached_) {
    getRobotPose(cur_pose);
    dist = getDistanceXY(cur_pose, global_plan_.back());
    // std::cout << "dist to goal: " << dist << '\n';
    if (dist < 0.8) {
      std::cout << ">>>> goal reached\n";
      goal_reached_ = true;
    }
  }

  return goal_reached_;
}

bool SimpleLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  if (locked_by_man_)
    return true;

  cmd_vel.linear.x = oa_vel_vx_;  // centermeter to m
  cmd_vel.angular.z = oa_vel_rz_; // radians to degree 
  return true;
}

}
