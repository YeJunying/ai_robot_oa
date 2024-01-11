#include "movebase_client.h"

#include <iostream>
#include <mutex>

#include "bluetooth.h"
#include "globals.h"
#include "man_control.h"

namespace ai_robot {

MoveBaseClient::MoveBaseClient()
    : nh_(ros::NodeHandle()),
      ac_(nullptr),
      state_(MoveBaseTaskState::kNotStarted) {
  // mapdata_srv_ = nh_.serviceClient<nav_msgs::GetMap>("/static_map");
  // initpose_sub_ = nh_.subscribe("/initialpose", 1, &MoveBaseClient::initialPoseCb, this);
  // goal_sub_ = nh_.subscribe("move_base_simple/goal", 1, &MoveBaseClient::movebaseGoalCb, this);
  manctl_state_sub_ = nh_.subscribe("ai_robot/manctl_state", 1, &MoveBaseClient::manctlStateCb, this);
  ac_ = new MoveBaseActionClient(nh_, "ai_robot/movebase", true);
  // getMapInfo();
  ac_->waitForServer();
}

MoveBaseClient::~MoveBaseClient() {
  if (ac_ != nullptr)
    delete ac_;
}

void MoveBaseClient::manctlStateCb(const std_msgs::UInt8ConstPtr& msg) {
  switch (static_cast<ManControlState>(msg->data)) {
    case ManControlState::kCancelled:
      ROS_INFO_NAMED("movebase_client", "Task cancelled by man");
      cancelTask();
      break;
    default:
      break;
  }
}

// void MoveBaseClient::initialPoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
//   start_pos_(0) = msg->pose.pose.position.x - mapinfo_.origin.x;
//   start_pos_(1) = msg->pose.pose.position.y - mapinfo_.origin.y;
//   BluetoothTransform::plane2merca(start_pos_);
// }

void MoveBaseClient::doTask(const geometry_msgs::PoseStamped& goal) {
  std::lock_guard<std::mutex> lock(g::mtx);
  g::in_man_mode = false;
  state_ = MoveBaseTaskState::kNotStarted;
  ai_robot_msgs::MoveBaseGoal action_goal;
  action_goal.target_pose = goal;
  action_goal.target_pose.header.stamp = ros::Time::now();
  ac_->sendGoal(action_goal,
               boost::bind(&MoveBaseClient::doneCb, this, _1, _2),
               boost::bind(&MoveBaseClient::activeCb, this),
               boost::bind(&MoveBaseClient::feedbackCb, this, _1));
}

void MoveBaseClient::cancelTask() {
  ac_->cancelGoal();
}

void MoveBaseClient::doneCb(const actionlib::SimpleClientGoalState& state, const ai_robot_msgs::MoveBaseResultConstPtr& result) {
  state_ = MoveBaseTaskState::kDone;
  ROS_INFO_NAMED("movebase_client", "Finished in state [%s]", state.toString().c_str());
  // ROS_INFO_NAMED("movebase_client", "Result: %d", result->success);
  std::lock_guard<std::mutex> lock(g::mtx);
  g::in_man_mode = true;
}

void MoveBaseClient::activeCb() {
  state_ = MoveBaseTaskState::kActive;
  ROS_INFO_NAMED("movebase_client", "Goal just went active");
}

void MoveBaseClient::feedbackCb(const ai_robot_msgs::MoveBaseFeedbackConstPtr& feedback) {
  // ROS_INFO_NAMED("movebase_client", "Got feedback from task: x=%.4f, y=%.4f",
  //                feedback->base_position.pose.position.x,
  //                feedback->base_position.pose.position.y);
}

}
