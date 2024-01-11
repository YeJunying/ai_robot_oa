#ifndef AI_ROBOT_BASE_CONTROL_H
#define AI_ROBOT_BASE_CONTROL_H

#include <cmath>

#include <ros/ros.h>

namespace ai_robot {

class BaseControl {
  static constexpr float kMaxLinearVel = 0.5;
  static constexpr float kMaxAngularVel = M_PI / 6;
public:
  static BaseControl& getInstance();
  BaseControl(const BaseControl&) = delete;
  BaseControl(BaseControl&&) = delete;
  BaseControl& operator=(const BaseControl&) = delete;
  BaseControl& operator=(BaseControl&&) = delete;
  virtual void move(float surge, float sway, float heave, float yaw);
  virtual void hover();
  virtual void enable();
  virtual void disable();

private:
  BaseControl();
  virtual ~BaseControl();

private:
  ros::Publisher vel_pub_;
  // ros::Subscriber enable_srv_;
  // ros::Subscriber disable_srv_;
  ros::NodeHandle nh_;
};

}

#endif // !AI_ROBOT_BASE_CONTROL_H
