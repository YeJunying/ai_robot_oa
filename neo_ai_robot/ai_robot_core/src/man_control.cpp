#include "man_control.h"

#include <cmath>
#include <cstdint>

#include "base_control.h"
#include "globals.h"

namespace ai_robot {

constexpr float kButtonHoldThresh {0.5};
constexpr float kHandleHoldThresh {0.01};

static inline bool checkHandleHoldThresh(float input) {
  return std::fabs(input) > kHandleHoldThresh;
}

ManControl::ManControl()
    : state_(ManControlState::kUnlocked),
      surge_(0.0),
      sway_(0.0),
      heave_(0.0),
      yaw_(0.0) {
  ros::NodeHandle nh;
  joy_sub_ = nh.subscribe("joy", 1, &ManControl::joyCb, this);
  joystate_pub_ = nh.advertise<std_msgs::UInt8>("ai_robot/manctl_state", 1, true);
  // vel_pub_ = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
}

ManControl::~ManControl() {}

void ManControl::joyCb(const sensor_msgs::Joy::ConstPtr& msg) {
  // Buttons
  if (msg->buttons[0] > kButtonHoldThresh && state_ == ManControlState::kUnlocked) { // Button A
    BaseControl::getInstance().hover();
    // Notify other nodes like local planner with the state of man locked.
    state_ = ManControlState::kLocked;
    state_msg_.data = static_cast<uint8_t>(state_);
    joystate_pub_.publish(state_msg_);
    std::lock_guard<std::mutex> lock(g::mtx);
    g::in_man_mode = true;
    return;
  } else if (msg->buttons[1] > kButtonHoldThresh && state_ == ManControlState::kLocked) { // Button B
    state_ = ManControlState::kUnlocked;
    state_msg_.data = static_cast<uint8_t>(state_);
    joystate_pub_.publish(state_msg_);
    std::lock_guard<std::mutex> lock(g::mtx);
    g::in_man_mode = true;
    return;
  } else if (msg->buttons[2] > kButtonHoldThresh) { // Button X
    state_ = ManControlState::kCancelled;
    state_msg_.data = static_cast<uint8_t>(state_);
    joystate_pub_.publish(state_msg_);
    std::lock_guard<std::mutex> lock(g::mtx);
    g::in_man_mode = true;
    return;
  }

  // std::lock_guard<std::mutex> lock(g::mtx);
  if (g::in_man_mode) {
    // Handles
    sway_ = msg->axes[0];
    surge_ = msg->axes[1];
    yaw_ = msg->axes[3];
    heave_ = msg->axes[4];
    if (checkHandleHoldThresh(surge_) || checkHandleHoldThresh(sway_) ||
        checkHandleHoldThresh(heave_) || checkHandleHoldThresh(yaw_)) {
      return;
    }
  }
}

void ManControl::control() {
  if (state_ == ManControlState::kLocked && !g::in_man_mode) {
    BaseControl::getInstance().hover();
    ROS_INFO("Locked, changed to man control mode.");
    std::lock_guard<std::mutex> lock(g::mtx);
    g::in_man_mode = true;
    return;
  } else if (state_ == ManControlState::kUnlocked && g::in_man_mode) {
    ROS_INFO("Unlocked, changed to auto control mode.");
    std::lock_guard<std::mutex> lock(g::mtx);
    g::in_man_mode = false;
    return;
  } else if (state_ == ManControlState::kCancelled && !g::in_man_mode) {
    ROS_INFO("Cancelled, changed to man control mode.");
    std::lock_guard<std::mutex> lock(g::mtx);
    g::in_man_mode = true;
    return;
  }

  // ROS_INFO("Handle control");
  // ROS_INFO("robot move: %f, %f, %f, %f", surge_, sway_, heave_, yaw_);
  BaseControl::getInstance().move(surge_, sway_, heave_, yaw_);

  if (std::fabs(surge_) + std::fabs(sway_) + std::fabs(heave_) + std::fabs(yaw_) < 0.01)
    BaseControl::getInstance().hover();
}

}

