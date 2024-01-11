#ifndef AI_ROBOT_SIMPLE_LOCAL_PLANNER_H
#define AI_ROBOT_SIMPLE_LOCAL_PLANNER_H

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

#include "ai_robot_msgs/OAVelocity.h"

#include "mapinfo.h"

namespace ai_robot {

typedef struct {
  double v[3] = {0};
} pose_t;

typedef struct {
  double v[2] = {0};
} pos_t;

class SimpleLocalPlanner {
public:
  SimpleLocalPlanner(MapInfo *mapinfo);
  virtual ~SimpleLocalPlanner();

public:
  void initialize(std::string name, tf2_ros::Buffer *tf);
  bool isInitialized() { return initialized_; }
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
  bool computeVelocityCommands1(geometry_msgs::Twist& cmd_vel);
  bool isGoalReached();
  bool isLockedByMan() { return locked_by_man_; }

private:
  bool getRobotPose(geometry_msgs::PoseStamped& global_pose);
  // void initialPoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
  void blueCB(const geometry_msgs::PointStampedConstPtr& msg);
  void imuCB(const sensor_msgs::ImuConstPtr& msg);
  void odomCB(const nav_msgs::OdometryConstPtr& msg);
  // void handleOdomAndBluetooth(const nav_msgs::OdometryConstPtr& odom_msg, const geometry_msgs::PointStampedConstPtr& blue_msg);
  void velocityCB(const ai_robot_msgs::OAVelocityConstPtr& msg);
  void manctlStateCB(const std_msgs::UInt8ConstPtr& msg);
  double getDistanceXY(const geometry_msgs::Pose& p1, const geometry_msgs::PoseStamped& p2);
  double getDistanceXY(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);
  double getDistanceXY(double x1, double x2, double y1, double y2);
  double getAngleDifference(double new_hdg, double cur_hdg);
  // double distanceToDistination(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);
  // Eigen::Vector2d getPositionFromOdomToMercator(const Eigen::Vector2d& pos);
  // Eigen::Vector2d getPositionFromMercatorToOdom(const Eigen::Vector2d& pos);
  // Eigen::Vector2d getPositionFromMercatorToMap(const Eigen::Vector2d& pos);
  // Eigen::Vector2d getPositionFromMapToMercator(const Eigen::Vector2d& pos);
  // void transformPositionFromOdomToMercator(const Eigen::Vector2d& in, Eigen::Vector2d& out);
  // void transformPositionFromMercatorToOdom(const Eigen::Vector2d& in, Eigen::Vector2d& out);
  void transformPositionFromMercatorToMap(const Eigen::Vector2d& in, Eigen::Vector2d& out);
  void transformPositionFromMercatorToMap(double in_x, double in_y, double& out_x, double& out_y);
  // void transformPositionFromMapToMercator(const Eigen::Vector2d& in, Eigen::Vector2d& out);
  void updateHeading(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);
  // void getMapInfo();
  double transformHeadingFromNeuToMap(double hdg);
  double transformHeadingFromMapToNeu(double hdg);
  double transformHeadingFromNwuToMap(double hdg);
  double transformHeadingFromMapToNwu(double hdg);
  double transformHeadingFromNwuToNeu(double hdg);
  double transformHeadingFromNeuToNwu(double hdg);
  
private:
  bool initialized_;
  bool locked_by_man_;
  bool imu_has_mag_;
  // Eigen::Vector2d cur_pos_;
  // geometry_msgs::PoseStamped cur_pose_to_origin_;
  // geometry_msgs::PoseStamped dst_pose_to_origin_;
  // geometry_msgs::PoseStamped cur_waypoint_to_origin_;

  // Rotation matrix from odom to earth (mercator)
  // Eigen::Matrix2d rotm_odom2merca_;
  // Eigen::Vector2d tr_odom2merca_;

  // Pose
  double initial_hdg_;
  double hdg_;
  double new_hdg_;
  Eigen::Vector2d pos_;
  Eigen::Vector2d blue_pos_;
  Eigen::Vector2d waypoint_in_merca_;
  geometry_msgs::PoseStamped odom_pose_;

  // pose_t initial_pose_;
  // pose_t cur_pose_;

  // Constraints/Limits
  double max_vel_x_;
  double max_vel_y_;
  double max_vel_theta_;
  double robot_width_;
  double robot_safe_gap_;
  double xy_goal_tolerance_blue_;
  double xy_goal_tolerance_;
  double yaw_goal_tolerance_;
  double imu_hdg_offset_to_map_;

  // Plan
  bool has_new_plan_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  // std::vector<geometry_msgs::PoseStamped> global_plan_map_;
  // std::vector<geometry_msgs::PoseStamped> global_plan_odom_;
  // std::vector<geometry_msgs::PoseStamped> local_plan_;
  // std::vector<geometry_msgs::PoseStamped> waypoints_in_odom_;
  size_t from_index_, to_index_;
  double odom_dist_to_waypoint_;
  double blue_dist_to_waypoint_;
  double next_dist_from_odom_;
  bool waypoint_reached_;
  bool turn_done_;
  bool new_hdg_updated_;
  bool goal_reached_;

  // publishers/subscribers
  ros::Subscriber imu_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber blue_sub_;
  ros::Subscriber manctl_state_sub_;
  tf2_ros::Buffer *tf_;
  // tf2_ros::TransformListener tfl_;

  MapInfo *mapinfo_;
  // ros::ServiceClient mapinfo_srv_;

  double defl_from_neu_to_map_; // deflection from ENU frame (mercator) to map frame
  double defl_from_nwu_to_map_; // deflection from NWU frame (magnetometer) to map frame
  bool is_merca_aligned_to_map_; // whether mercator frame is rotated to align with map frame

  // for communication with oa python node
  double oa_vel_vx_;
  double oa_vel_rz_;
  double guide_angle_;
  ros::Subscriber oa_vel_sub_;
  ros::ServiceClient oa_pose_srv_, oa_tgt_srv_;

  double transform_tolerance_;
};

}

#endif // !AI_ROBOT_SIMPLE_LOCAL_PLANNER_H#
