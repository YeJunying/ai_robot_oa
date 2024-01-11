#ifndef AI_ROBOT_APF_LOCAL_PLANNER_H
#define AI_ROBOT_APF_LOCAL_PLANNER_H

#include <string>
#include <vector>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt8.h>
#include <tf2_ros/buffer.h>

#include "mapinfo.h"

class ApfLocalPlanner {
public:
  ApfLocalPlanner(MapInfo *mapinfo);
  virtual ~ApfLocalPlanner();

public:
  void initialize(std::string name, tf2_ros::Buffer *tf);
  bool isInitialized() { return initialized_; }
  bool setPlan(const std::vector<goemetry_msgs::PoseStamped>& orig_global_plan);
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
  bool isGoalReached() { return goal_reached_; }
  bool isLockedByMan() { return locked_by_man_; }

private:
  bool initialized_;
  bool locked_by_man_;

  // Plan
  bool goal_reached_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;

  MapInfo *mapinfo_;
}


#endif // !AI_ROBOT_SIMPLE_LOCAL_PLANNER_H#
