#ifndef AI_ROBOT_JOY_CONTROL_H
#define AI_ROBOT_JOY_CONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt8.h>

#include "globals.h"

namespace ai_robot {

enum class ManControlState {
  kUnlocked,
  kLocked,
  kCancelled
};

class ManControl {
public:
  ManControl();
  virtual ~ManControl();

public:
  void control();

private:
  void joyCb(const sensor_msgs::Joy::ConstPtr& msg);

private:
  float surge_, sway_, heave_, yaw_;
  ManControlState state_;
  std_msgs::UInt8 state_msg_;
  ros::Subscriber joy_sub_;
  ros::Publisher joystate_pub_;
};

}


#endif // !AI_ROBOT_JOY_CONTROL_H
