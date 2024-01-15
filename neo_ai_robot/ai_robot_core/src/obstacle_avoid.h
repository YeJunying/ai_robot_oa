#ifndef AI_ROBOT_OBSRACLE_AVOID_H
#define AI_ROBOT_OBSRACLE_AVOID_H

#include <string>
#include <vector>

#include <Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/UInt8.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/exceptions.h>

#include "ai_robot_msgs/OAVelocity.h"

#include "octree.h"
#include "mapinfo.h"

namespace ai_robot {

typedef struct {
  double v[3] = {0};
} pose_t;

typedef struct {
  double v[2] = {0};
} pos_t;

struct SafeZonePoint {
  float dist;
  float angle;
};

struct SafeZone {
  SafeZone() : near(nullptr), far(nullptr){};
  virtual ~SafeZone() = default;
  float getDirectionDiff(float orig_dir, float orig_dist, float& dir, float& dist);
  void setSimpleSafeZone(float ldist, float langle, float rdist, float rangle, int llevel, int rlevel, float dia);

  SafeZonePoint left, right;
  float left_dir, right_dir;
  float* near;
  float* far;
};

class ObstacleAvoid {
public:
  ObstacleAvoid(MapInfo *mapinfo);
  virtual ~ObstacleAvoid();

public:
  void initialize(std::string name, tf2_ros::Buffer *tf);
  bool isInitialized() { return initialized_; }
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
  bool isGoalReached();
  bool isLockedByMan() { return locked_by_man_; }
  void safeZoneView();
  void getLocalTargetPose(geometry_msgs::PoseStamped &cur_pose);

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
  void transformPositionFromMercatorToMap(const Eigen::Vector2d& in, Eigen::Vector2d& out);
  void transformPositionFromMercatorToMap(double in_x, double in_y, double& out_x, double& out_y);
  void updateHeading(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);
  double transformHeadingFromNeuToMap(double hdg);
  double transformHeadingFromMapToNeu(double hdg);
  double transformHeadingFromNwuToMap(double hdg);
  double transformHeadingFromMapToNwu(double hdg);
  double transformHeadingFromNwuToNeu(double hdg);
  double transformHeadingFromNeuToNwu(double hdg);
  void updateVelocityToLocalTarget(bool findpath_flag);
  void modifVelocity(double tgt_dist, double dyaw, bool findpath_flag);
  void modifVelocityByLidar(double tgt_dist, double dyaw);
  void smoothVelocity();
  void lidarCallback(const sensor_msgs::LaserScanConstPtr& msg);
  void findSafeZonesWithOctree(const sensor_msgs::LaserScanConstPtr& msg);
  int findNearestSafeZone(float orig_dir, float tgt_dist, float& oa_dir_diff, float& oa_dist, float& oa_dir,
                          float& body_dir_diff, bool& in_safezone);
  void showNearestSafezone(float, int);
  void map2Baselink(geometry_msgs::Pose& loc_tgt);
  void readParameters();
  void initPublishesAndSubscribes();
  
private:
  bool initialized_;
  bool locked_by_man_;
  bool imu_has_mag_;
  // Pose
  double initial_hdg_;
  double hdg_;
  double new_hdg_;
  Eigen::Vector2d pos_;
  Eigen::Vector2d blue_pos_;
  Eigen::Vector2d waypoint_in_merca_;
  geometry_msgs::PoseStamped odom_pose_;

  // Constraints/Limits
  double max_vel_x_;
  double max_vel_y_;
  double max_vel_theta_;
  double xy_goal_tolerance_blue_;
  double xy_goal_tolerance_;
  double yaw_goal_tolerance_;
  double imu_hdg_offset_to_map_;

  // parameters
  double a_theta_;
  double b_alpha_;
  double c_dist_;
  double max_vx_;
  double max_rz_;
  double min_turn_radius_;
  double obst_det_thresh_;
  double robot_width_;
  double robot_safe_gap_;
  double safezone_max_width_;
  double max_direction_diff_;
  double safezone_det_thresh_;
  double abs_safe_dist_;
  double half_fov_;
  bool lidar_used_;
  bool is_view_safezone_lidar_;
  double number_of_steps_;
  double range_step_size_;
  double angle_step_;
  double neighbor_search_diameter_;
  double valid_half_pi_;
  double tr_from_base_to_lidar_[3];
  double deadzone_radius_;

  // Plan
  bool has_new_plan_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  size_t from_index_, to_index_;
  double odom_dist_to_waypoint_;
  double blue_dist_to_waypoint_;
  double next_dist_from_odom_;
  bool waypoint_reached_;
  bool turn_done_;
  bool new_hdg_updated_;
  bool goal_reached_;

  // publishers/subscribers
  ros::Subscriber lidar_sub_;
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

  // lidar
  pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_pcl_xyz_;
  unibn::Octree<pcl::PointXYZ> lidar_xyz_oct_;
  std::vector<SafeZone> safezones_;
  double max_valid_half_fov_;
  float center_dir_dist_;

  // smooth
  float last_vx_;
  float last_rz_;

  // view safezone
  cv::Mat safezone_view_;
  float mod_oa_dir_;

  // for replan
  geometry_msgs::PoseStamped local_tgt_pose_;
  geometry_msgs::PoseStamped global_tgt_pose_;
  pcl::PointXYZ local_tgt_pclxyz_;
  int tgt_point_blocked_;

  // time
  double time_intvl2_;
};

}

#endif // !AI_ROBOT_OBSRACLE_AVOID_H_
