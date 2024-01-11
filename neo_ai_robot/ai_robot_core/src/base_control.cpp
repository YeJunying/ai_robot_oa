#include "base_control.h"

#include <ros/console.h>
#include <geometry_msgs/Twist.h>

namespace ai_robot {

BaseControl::BaseControl() : nh_(ros::NodeHandle()) {
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  // enable_srv_ = nh_.serviceClient<std_srvs::Empty>("enable_motors");
  // disable_srv_ = nh_.serviceClient<std_srvs::Empty>("disable_motors");
}

BaseControl::~BaseControl() {
}

BaseControl& BaseControl::getInstance() {
  static BaseControl instance;
  return instance;
}

void BaseControl::move(float surge, float sway, float heave, float yaw) {
  if (surge > 1.01)
    surge = 1;
  else if (surge < -1.01)
    surge = -1;
  if (sway > 1.01)
    sway = 1;
  else if (sway < -1.01)
    sway = -1;
  if (heave > 1.01)
    heave = 1;
  else if (heave < -1.01)
    heave = -1;
  if (yaw > 1.01)
    yaw = 1;
  else if (yaw < -1.01)
    yaw = -1;

  geometry_msgs::Twist vel;

  vel.linear.x = surge * BaseControl::kMaxLinearVel;
  vel.linear.y = 0;
  vel.linear.z = 0;
  vel.angular.x = 0;
  vel.angular.y = 0;
  vel.angular.z = yaw * BaseControl::kMaxAngularVel;

  ROS_DEBUG_NAMED("base_control", "Publish vel");
  vel_pub_.publish(vel);
}

void BaseControl::hover() {
  move(0, 0, 0, 0);
}

void BaseControl::enable() {
  ROS_WARN_NAMED("base_control", "Function enable() is not implemented");
}

void BaseControl::disable() {
  ROS_WARN_NAMED("base_control", "Function disable() is not implemented");
}

}
