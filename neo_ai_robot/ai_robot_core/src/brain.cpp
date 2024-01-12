#include "brain.h"
#include "globals.h"

#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetMap.h>

#include "movebase_client.h"

#include "bluetooth.h"

namespace ai_robot {

// Brain::Brain(const ros::NodeHandle& nh) : nh_(nh), simplemove_client_(nh) {
Brain::Brain() :
    nh_(ros::NodeHandle()),
    taskid_(TaskId::kIdle),
    movebase_task_state_(MoveBaseTaskState::kNotStarted) {
  // ros::NodeHandle private_nh("~");
  movebase_client_ = new MoveBaseClient();
  manctl_ = new ManControl();
  goal_sub_ = nh_.subscribe("move_base_simple/goal", 1, &Brain::moveBaseGoalCb, this);
  getMapInfo();
  ROS_INFO("Everything is Ready.");
}

Brain::~Brain() {
  if (movebase_client_ != nullptr)
    delete movebase_client_;
  if (manctl_ != nullptr)
    delete manctl_;
}

void Brain::getMapInfo() {
  mapinfo_srv_ = nh_.serviceClient<nav_msgs::GetMap>("/static_map");
  nav_msgs::GetMap map;
  mapinfo_srv_.call(map);
  mapinfo_.resolution = map.response.map.info.resolution;
  mapinfo_.origin.x = map.response.map.info.origin.position.x;
  mapinfo_.origin.y = map.response.map.info.origin.position.y;
}

void Brain::moveBaseGoalCb(const geometry_msgs::PoseStampedConstPtr& msg) {
  // geometry_msgs::PoseStamped goal_pose_in_map = *msg;
  // Converts goal position in map to left bottom to match bluetooth frame.
  // ROS_INFO("goal position in map: %.2f, %.2f", goal_msg_.)
  // goal_msg_.pose.position.x = msg->pose.position.x - mapinfo_.origin.x;
  // goal_msg_.pose.position.y = msg->pose.position.y - mapinfo_.origin.y;
  goal_msg_.pose.position.x = msg->pose.position.x;
  goal_msg_.pose.position.y = msg->pose.position.y;
  goal_msg_.pose.orientation.x = msg->pose.orientation.x;
  goal_msg_.pose.orientation.y = msg->pose.orientation.y;
  goal_msg_.pose.orientation.z = msg->pose.orientation.z;
  goal_msg_.pose.orientation.w = msg->pose.orientation.w;
  // BluetoothTransform::plane2merca(goal_msg_.pose.position.x, goal_msg_.pose.position.y,
  //                                 goal_msg_.pose.position.x, goal_msg_.pose.position.y);

  // doTask(goal_msg_);
  movebase_client_->doTask(goal_msg_);
  if (movebase_client_->getState() == MoveBaseTaskState::kActive)
    taskid_ = TaskId::kMoveBase;
}

void Brain::think() {
  static bool first_run = true;
  if (g::in_man_mode)
    taskid_ = TaskId::kManControl;

  switch (taskid_) {
    case TaskId::kManControl:
      manctl_->control();
      break;
    case TaskId::kMoveBase:
      if (movebase_client_->getState() == MoveBaseTaskState::kDone || movebase_client_->getState() == MoveBaseTaskState::kCancelled)
        taskid_ = TaskId::kIdle;
      break;
    default:
      break;
  }
  // if (task_id_ != TaskId::kMoveBase && first_run && !g::man_test_mode) { // Ensure to run only once on development stage
  //   first_run = false;
  //   task_id_ = TaskId::kMoveBase;
  //   testMoveBaseTask();
  // }
}

// Fixes start point in test
void Brain::testMoveBaseTask() {
  geometry_msgs::PoseStamped goal;
  goal.header.stamp = ros::Time::now();
  goal.pose.position.x = 13519950.096968763;
  goal.pose.position.y = 3614355.5299259643;
  movebase_client_->doTask(goal);
}

}
