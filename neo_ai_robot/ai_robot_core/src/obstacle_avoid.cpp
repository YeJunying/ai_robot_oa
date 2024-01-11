#include "obstacle_avoid.h"

#include <cmath>
#include <thread>

#include <chrono>

#include <cstdint>
#include <cstdio>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <math.h>
#include <nav_msgs/GetMap.h>
#include <ros/node_handle.h>
#include <std_msgs/Float32MultiArray.h>
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

constexpr int kImgRow{250};
constexpr int kImgCol{500};
constexpr float kRangeThresh {10};
const unibn::OctreeParams kOctreeParams {unibn::OctreeParams(10, false, 0.1f)};

static void pol2cart(float r, float theta, float& x, float& y) {
  x = r * std::cos(theta);
  y = r * std::sin(theta);
}

static void cart2pol(float x, float y, float& r, float& theta) {
  r = std::hypot(x, y);
  theta = std::atan2(y, x);
}

// @orig_dir: original direction from the robot body to the local target
// @orig_dist: original distance from the robot body to the local target
// @dir: rectified direction
// @dist: rectified distance
float SafeZone::getDirectionDiff(float orig_dir, float orig_dist, float& dir, float& dist) {
  if (right.dist < 0 || left.dist < 0) {
    return -10.0;
  }

  float ret;

  if (orig_dir >= right_dir && orig_dir <= left_dir) {  // within the left and right angles
    dist = (left.dist + right.dist) / 2;
    dir = orig_dir;
    ret = .0;
  } else {  // not in safezone
    dir = (left_dir + right_dir) / 2;
    dist = (orig_dir < right_dir) ? right.dist : left.dist;
    ret = dir - orig_dir;
  }
  return ret;
}

void SafeZone::setSimpleSafeZone(float ldist, float langle, float rdist, float rangle, int llevel, int rlevel,
                                 float dia) {
  left.dist = ldist < 20 ? ldist : 20;
  left.angle = langle;
  right.dist = rdist < 20 ? rdist : 20;
  right.angle = rangle;

  right_dir = rangle + std::acos(1 - std::pow(0.15 / dia / (rlevel + 1), 2) / 2.3);
  left_dir = langle - std::acos(1 - std::pow(0.15 / dia / (llevel + 1), 2) / 2.3);
  if (right_dir > left_dir) {
    left_dir = right_dir = (langle + rangle) / 2;
  }
}

ObstacleAvoid::ObstacleAvoid(MapInfo *mapinfo)
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
      transform_tolerance_(0.5),
      lidar_pcl_xyz_(new pcl::PointCloud<pcl::PointXYZ>), 
      lidar_used_(false), 
      tgt_point_blocked_(1),
      last_rz_(0.0),
      last_vx_(0.0),
      center_dir_dist_(2){
      // buffer_(new tf2_ros::Buffer),
      // listener_(new tf2_ros::TransformListener(buffer_)) {
  readParameters();
  initPublishesAndSubscribes();
  
  if(is_view_safezone_lidar_ && lidar_used_)
    safezone_view_ = cv::Mat(kImgRow, kImgCol, CV_8UC3, cv::Scalar(255, 255, 255)).clone();
}

ObstacleAvoid::~ObstacleAvoid() { }

void ObstacleAvoid::transformPositionFromMercatorToMap(const Eigen::Vector2d& in, Eigen::Vector2d& out) {
  bluetooth::Transform::merca2plane(in(0), in(1), out(0), out(1), mapinfo_->resolution);
  out[0] += mapinfo_->origin.x;
  out[1] += mapinfo_->origin.y;
}

void ObstacleAvoid::transformPositionFromMercatorToMap(double in_x, double in_y, double& out_x, double& out_y) {
  bluetooth::Transform::merca2plane(in_x, in_y, out_x, out_y, mapinfo_->resolution);
  out_x += mapinfo_->origin.x;
  out_y += mapinfo_->origin.y;
}

double ObstacleAvoid::transformHeadingFromNeuToMap(double hdg) {
  return util::normAngle(hdg - defl_from_neu_to_map_);
}

double ObstacleAvoid::transformHeadingFromMapToNeu(double hdg) {
  return util::normAngle(hdg + defl_from_neu_to_map_);
}

double ObstacleAvoid::transformHeadingFromNwuToMap(double hdg) {
  return util::normAngle(hdg - defl_from_nwu_to_map_);
}

double ObstacleAvoid::transformHeadingFromMapToNwu(double hdg) {
  return util::normAngle(hdg + defl_from_nwu_to_map_);
}

double ObstacleAvoid::transformHeadingFromNwuToNeu(double hdg) {
  return util::normAngle(hdg + M_PI_2);
}

double ObstacleAvoid::transformHeadingFromNeuToNwu(double hdg) {
  return util::normAngle(hdg - M_PI_2);
}

double ObstacleAvoid::getDistanceXY(const geometry_msgs::Pose& p1, const geometry_msgs::PoseStamped& p2) {
  return std::hypot(p1.position.x - p2.pose.position.x, p1.position.y - p2.pose.position.y);
}

double ObstacleAvoid::getDistanceXY(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2) {
  return std::hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
}

double ObstacleAvoid::getDistanceXY(double x1, double y1, double x2, double y2) {
  return std::hypot(x2 - x1, y2 - y1);
}

// Working directly with mercator
double ObstacleAvoid::getAngleDifference(double new_hdg, double cur_hdg) {
  double yaw = util::getAngleDiff(new_hdg, cur_hdg); 
  return util::normAngle(yaw);
}

void ObstacleAvoid::initialize(std::string name, tf2_ros::Buffer *tf) {
 if (!isInitialized()) {
    ros::NodeHandle private_nh("~/" + name);
    ros::NodeHandle nh;
    // oa_pose_srv_ = nh.serviceClient<ai_robot_msgs::SetRobotPose>("/oa_robot_pose");
    // oa_tgt_srv_ = nh.serviceClient<ai_robot_msgs::SetTargetPosition>("/oa_target_pos");
    imu_sub_ = nh.subscribe("/imu", 1, &ObstacleAvoid::imuCB, this);
    odom_sub_ = nh.subscribe("/odom", 1, &ObstacleAvoid::odomCB, this);
    blue_sub_ = nh.subscribe("/blue_pose", 1, &ObstacleAvoid::blueCB, this);
    manctl_state_sub_ = nh.subscribe("ai_robot/manctl_state", 1, &ObstacleAvoid::manctlStateCB, this);
    goal_reached_ = false;
    waypoint_reached_ = false;
    tf_ = tf;

    private_nh.param("defl_from_neu_to_map", defl_from_neu_to_map_, 19.0);
    private_nh.param("defl_from_neu_to_map", defl_from_nwu_to_map_, -71.0);
    private_nh.param("is_mercator_aligned_to_map", is_merca_aligned_to_map_, false);
    // private_nh.param("max_vel_x", max_vel_x_, 0.5);
    // private_nh.param("max_vel_y", max_vel_y_, 0.25);
    // private_nh.param("max_vel_theta", max_vel_theta_, 1.87);
    private_nh.param("xy_goal_tolerance_blue", xy_goal_tolerance_blue_, 2.5);
    private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 1.0);
    private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
    // private_nh.param("robot_width", robot_width_, 0.15);
    // private_nh.param("robot_safe_gap", robot_safe_gap_, 0.2);
    private_nh.param("imu_hdg_offset_to_map", imu_hdg_offset_to_map_, M_PI_2 + 19.0);
    private_nh.getParam("imu_has_mag", imu_has_mag_);
    defl_from_neu_to_map_ = util::deg2rad(defl_from_neu_to_map_);
    defl_from_nwu_to_map_ = util::deg2rad(defl_from_nwu_to_map_);

  }
}

void ObstacleAvoid::getLocalTargetPose(geometry_msgs::PoseStamped &cur_pose) {
  tf2::toMsg(tf2::Transform::getIdentity(), cur_pose.pose);
  geometry_msgs::PoseStamped tmp_pose;
  tmp_pose.pose = global_tgt_pose_.pose;
  tmp_pose.header.frame_id = "map";
  tmp_pose.header.stamp = ros::Time();
  ros::Time curr_time = ros::Time::now();

  try {
    tf_->transform(tmp_pose, cur_pose, "base_link");
  } catch (tf2::LookupException& ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }
}

bool ObstacleAvoid::getRobotPose(geometry_msgs::PoseStamped& global_pose) {
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

// void ObstacleAvoid::map2Baselink(geometry_msgs::Pose& local_tgt_pose) {
//   tf2::toMsg(tf2::Transform::getIdentity(), local_tgt_pose);
//   geometry_msgs::PoseStamped robot_pose;
//   tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
//   robot_pose.header.frame_id = "base_link";
//   robot_pose.header.stamp = ros::Time();
//   ros::Time curr_time = ros::Time::now();
//
//   try {
//     tf_->transform(local_tgt_pose, robot_pose, "base_link");
//   } catch (tf2::LookupException& ex) {
//     ROS_ERROR_THROTTLE(1.0, "No Transform available Error lookup up robot pose: %s\n", ex.what());
//     return;
//   } catch (tf2::ConnectivityException& ex) {
//     ROS_ERROR_THROTTLE(1.0, "Connectivity Error lookup up robot pose: %s\n", ex.what());
//     return;
//   } catch (tf2::ExtrapolationException& ex) {
//     ROS_ERROR_THROTTLE(1.0, "Extrapolation Error lookup up robot pose: %s\n", ex.what());
//     return;
//   }
// }

void ObstacleAvoid::imuCB(const sensor_msgs::ImuConstPtr& msg) {
  double roll, pitch, yaw;
  util::quat2eul(roll, pitch, yaw, msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  hdg_ = imu_has_mag_ ? transformHeadingFromNwuToMap(yaw) : yaw;
}


// Working directly with mercator
void ObstacleAvoid::odomCB(const nav_msgs::OdometryConstPtr& msg) {
  double r, p, y;
  pos_[0] = msg->pose.pose.position.x;
  pos_[1] = msg->pose.pose.position.y;

  // odom_dist_to_waypoint_ = getDistanceXY(pos_[0], pos_[1], waypoint_in_map_[0], waypoint_in_map_[1]);
  double dist = getDistanceXY(pos_[0], pos_[1], 0, 0);
  odom_dist_to_waypoint_ = fabs(dist - next_dist_from_odom_);

}

// Directly working with mercator frame
void ObstacleAvoid::blueCB(const geometry_msgs::PointStampedConstPtr& msg) {
  // ROS_INFO("Bluetooth position: %.2f, %.2f", msg->point.x, msg->point.y);
  // Only within 3 meters to the next waypoint, consider bluetooth.
  blue_pos_[0] = msg->point.x;
  blue_pos_[1] = msg->point.y;

  if (!has_new_plan_)
    return;

  blue_dist_to_waypoint_ = getDistanceXY(blue_pos_[0], blue_pos_[1], global_plan_[to_index_].pose.position.x, global_plan_[to_index_].pose.position.y);
}

void ObstacleAvoid::velocityCB(const ai_robot_msgs::OAVelocityConstPtr& msg) {
  oa_vel_vx_ = msg->vx / 100; // cm -> m
  oa_vel_rz_ = util::deg2rad(msg->rz);  // degrees to radians
}

void ObstacleAvoid::manctlStateCB(const std_msgs::UInt8ConstPtr& msg) {
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

void ObstacleAvoid::updateHeading(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2) {
  new_hdg_ = std::atan2(p2.pose.position.y - p1.pose.position.y, p2.pose.position.x - p1.pose.position.x);
}

// Working directly with mercator frame
bool ObstacleAvoid::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
  global_plan_ = orig_global_plan;

  // NOTE: just for test!
  // hdg_ = M_PI;
  pos_[0] = global_plan_.front().pose.position.x;
  pos_[1] = global_plan_.front().pose.position.y;

  global_tgt_pose_.pose.position.x = pos_[0];
  global_tgt_pose_.pose.position.y = pos_[1];
  global_tgt_pose_.pose.orientation.x = global_plan_.front().pose.orientation.x;
  global_tgt_pose_.pose.orientation.y = global_plan_.front().pose.orientation.y;
  global_tgt_pose_.pose.orientation.z = global_plan_.front().pose.orientation.z;
  global_tgt_pose_.pose.orientation.w = global_plan_.front().pose.orientation.w;

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
  // oa_pose_srv_.waitForExistence();
  // oa_pose_srv_.call(robot_pose_msg);

  ai_robot_msgs::SetTargetPosition tgt_pos_msg;
  tgt_pos_msg.request.x = orig_global_plan[0].pose.position.x;
  tgt_pos_msg.request.y = orig_global_plan[0].pose.position.y;
  // oa_tgt_srv_.waitForExistence();
  // oa_tgt_srv_.call(tgt_pos_msg);

  goal_reached_ = false;

  return true;
}

bool ObstacleAvoid::isGoalReached() {
  static geometry_msgs::PoseStamped cur_pose;
  static double dist;

  if (!goal_reached_) {
    getRobotPose(cur_pose);
    dist = getDistanceXY(cur_pose, global_plan_.back());
    // std::cout << "dist to goal: " << dist << '\n';
    if (dist < 0.2) {
      ROS_INFO(">>>> goal reached");
      goal_reached_ = true;
    }
  }

  return goal_reached_;
}

bool ObstacleAvoid::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  if (locked_by_man_)
    return true;

  getLocalTargetPose(local_tgt_pose_);
  updateVelocityToLocalTarget(true);
  cmd_vel.linear.x = oa_vel_vx_;  // centermeter to m
  cmd_vel.angular.z = oa_vel_rz_; // radians to degree 
  return true;
}

// tgt_pos: local target pose
// return: if the distance to is less than 0.1, return false
void ObstacleAvoid::updateVelocityToLocalTarget(bool findpath_flag) {
  local_tgt_pclxyz_.x = local_tgt_pose_.pose.position.x;
  local_tgt_pclxyz_.y = local_tgt_pose_.pose.position.y;
  local_tgt_pclxyz_.z = local_tgt_pose_.pose.position.z;

  double tgt_dist = sqrt(pow(local_tgt_pose_.pose.position.x, 2) + pow(local_tgt_pose_.pose.position.y, 2));
  double dyaw = atan2(local_tgt_pose_.pose.position.y, local_tgt_pose_.pose.position.x);

  oa_vel_vx_ = tgt_dist / 4 * 5;
  oa_vel_vx_ = (oa_vel_vx_ > max_vx_) ? max_vx_ : oa_vel_vx_;

  oa_vel_rz_ = dyaw / M_PI_2 * max_rz_;
  if (oa_vel_rz_ > max_rz_)
    oa_vel_rz_ = max_rz_;
  else if (oa_vel_rz_ < -max_rz_)
    oa_vel_rz_ = -max_rz_;

  modifVelocity(tgt_dist, dyaw, findpath_flag);

  if (tgt_dist < 0.1) {
    oa_vel_vx_ = 0.0;
    oa_vel_rz_ = 0.0;
  }
}

void ObstacleAvoid::modifVelocity(double tgt_dist, double dyaw, bool findpath_flag) {
  if (lidar_used_) 
    modifVelocityByLidar(tgt_dist, dyaw);

  smoothVelocity();
  safeZoneView();
  if (!findpath_flag) {
    last_rz_ = 0;
    last_vx_ = 0;
  }
}

void ObstacleAvoid::modifVelocityByLidar(double tgt_dist, double dyaw) {
  if (oa_vel_vx_ <= 0) { // avoid moving backward
    oa_vel_vx_ = 0;
    return;
  }

  if (fabs(dyaw) > 0.7 * half_fov_) { // moving direction is out of detection
    oa_vel_vx_ = 0.05;
    return;
  }

  float oa_dir_diff;
  float oa_dist;
  float oa_dir;
  float body_dir_diff;
  bool in_safezone = false;
  int nearest_index = findNearestSafeZone(dyaw, tgt_dist, oa_dir_diff, oa_dist, oa_dir, body_dir_diff, in_safezone);

  if (nearest_index == -1 || oa_dir_diff > max_direction_diff_) {
    oa_vel_vx_ = 0.0;
    oa_vel_rz_ = 0.0;
  } else {
    float temp_vx;
    temp_vx = (center_dir_dist_ > 2) ? max_vx_ : (center_dir_dist_ * max_vx_ / 2.0);
    oa_vel_vx_ = (temp_vx < oa_vel_vx_) ? temp_vx : oa_vel_vx_;

    double tmp_yaw, tmp_pitch, tmp_roll;
    if (std::fabs(local_tgt_pose_.pose.orientation.x) + std::fabs(local_tgt_pose_.pose.orientation.y) + std::fabs(local_tgt_pose_.pose.orientation.z) +
        std::fabs(local_tgt_pose_.pose.orientation.w) < 0.1)
      tmp_yaw = 0;
    else
      util::quat2eul(tmp_roll, tmp_pitch, tmp_yaw, local_tgt_pose_.pose.orientation.w, local_tgt_pose_.pose.orientation.x, local_tgt_pose_.pose.orientation.y, local_tgt_pose_.pose.orientation.z);

    if (in_safezone)
      oa_vel_rz_ = tmp_yaw;
    else
      oa_vel_rz_ = (a_theta_ + b_alpha_) * oa_dir - b_alpha_ * tmp_yaw;

    if (std::fabs(oa_vel_vx_) > 0 && std::fabs(oa_vel_rz_) > 0) {
      float dvx = oa_dist / oa_vel_vx_;
      float drz = oa_dir / oa_vel_rz_;
      if (drz > dvx) 
        oa_vel_vx_ = oa_dist / drz;
    }

    if (oa_vel_vx_ > max_vx_)
      oa_vel_vx_ = max_vx_;
    else if (oa_vel_vx_ < 0.05)
      oa_vel_vx_ = 0.05;

    if (oa_vel_rz_ > max_rz_)
      oa_vel_rz_ = max_rz_;
    else if (oa_vel_rz_ < -max_rz_)
      oa_vel_rz_ = -max_rz_;

    if (min_turn_radius_ > 0 && std::fabs(oa_vel_rz_) != 0) {
      float turn_radius = std::fabs(oa_vel_vx_ / oa_vel_rz_);
      if (turn_radius < min_turn_radius_) {
        if (in_safezone || center_dir_dist_ > abs_safe_dist_)
          oa_vel_rz_ = oa_vel_vx_ / min_turn_radius_ * oa_vel_rz_ / std::fabs(oa_vel_rz_);
        else
          oa_vel_vx_ = 0;
      }
    }
  }
}

void ObstacleAvoid::findSafeZonesWithOctree(const sensor_msgs::LaserScanConstPtr& msg) {
  safezone_view_ = cv::Mat(kImgRow, kImgCol, CV_8UC3, cv::Scalar(255, 255, 255)).clone();
  cv::circle(safezone_view_, cv::Point(kImgCol / 2, kImgRow - 1), 50, cv::Scalar(100, 100, 0), 1);
  cv::circle(safezone_view_, cv::Point(kImgCol / 2, kImgRow - 1), 100, cv::Scalar(100, 100, 0), 1);
  cv::circle(safezone_view_, cv::Point(kImgCol / 2, kImgRow - 1), 150, cv::Scalar(100, 100, 0), 1);
  cv::circle(safezone_view_, cv::Point(kImgCol / 2, kImgRow - 1), 200, cv::Scalar(100, 100, 0), 1);
  cv::circle(safezone_view_, cv::Point(kImgCol / 2, kImgRow - 1), 400, cv::Scalar(100, 100, 0), 1);

  std::lock_guard<std::mutex> oa_mtx(g::oa_lock);
  safezones_.clear();
  lidar_pcl_xyz_->clear();
  

  pcl::PointXYZ p(0, 0, 0);
  float angle = msg->angle_min;  // -PI
  // Construct point cloud
  // We only need the scan data within the desired fov, i.e. -135 ~ 135 degrees.
  for (auto it = msg->ranges.begin(); it != msg->ranges.end() && angle <= max_valid_half_fov_;
       ++it, angle += msg->angle_increment) {
    if (angle >= -max_valid_half_fov_) {
      if (is_view_safezone_lidar_) {
        int imgx, imgy;
        // draw lidar points
        imgx = kImgCol / 2 - static_cast<int>(((*it) * sin(angle) + tr_from_base_to_lidar_[1]) * kImgRow / 5);
        imgy = kImgRow - 1 - static_cast<int>(((*it) * cos(angle) + tr_from_base_to_lidar_[0]) * kImgRow / 5);
        if (imgx >= 0 && imgx < kImgCol && imgy >= 0 && imgy < kImgRow) {
          cv::circle(safezone_view_, cv::Point(imgx, imgy), 1, cv::Scalar(0, 0, 0), 2);
        }
      }

      if (fabs(*it) < kRangeThresh) {
        pol2cart((*it), angle, p.x, p.y);
        lidar_pcl_xyz_->push_back(p);
      }
    }
  }

  // Construct octree from point cloud
  if (!lidar_pcl_xyz_->points.empty()) 
    lidar_xyz_oct_.initialize(lidar_pcl_xyz_->points, kOctreeParams);

  bool has_obst = false;  // has obstacle at current scan angle
  bool sn_start = false;
  std::vector<uint32_t> nbr_indices;  // neighbor indices
  float mid_angle = 1;
  int nsteps = 0;
  int cloest_dir_index = -1;
  float tgt_dir = std::atan2(local_tgt_pose_.pose.position.y, local_tgt_pose_.pose.position.x);
  float min_dir_diff = 100;
  float last_dist_level = -1;
  int llevel = 0, rlevel = 0;
  double last_safezone_leftbound = -M_PI;

  SafeZone safezone;
  SafeZonePoint start_point{2.03, 0}, end_point{2.03, 0};
  std::vector<int> all_levels;

  // Process each angle
  for (angle = -half_fov_; angle <= half_fov_; angle += angle_step_) {
    nbr_indices.clear();
    has_obst = false;
    float dx = range_step_size_ * std::cos(angle);
    float dy = range_step_size_ * std::sin(angle);
    float initial_x = deadzone_radius_ * std::cos(angle);  // in robot body (base_link) frame
    float initial_y = deadzone_radius_ * std::sin(angle);
    // transform to laser frame
    p.x = initial_x - tr_from_base_to_lidar_[0];
    p.y = initial_y - tr_from_base_to_lidar_[1];

    // Process each level on each angle
    for (nsteps = 0; nsteps < number_of_steps_; ++nsteps) {
      // If more than kObstacleDetectionThresh points found in the neighbor areas (a circle with diameter set to
      // neighbor_search_diameter_), then it is considered as a real obstacle, thus to filter out random noise.
      lidar_xyz_oct_.radiusNeighbors<unibn::L2Distance<pcl::PointXYZ>>(p, neighbor_search_diameter_ / 2, nbr_indices);
      if (nbr_indices.size() > obst_det_thresh_) {
        has_obst = true;
        break;
      }
      p.x += dx;
      p.y += dy;
    }

    // Angle within (-1, 1) radians is taken to calculate the distane along the center line (relative to robot body).
    // Always use the angle that is closer to the center line.
    if (std::fabs(angle) < mid_angle) {
      center_dir_dist_ = nsteps * range_step_size_;
      mid_angle = fabs(angle);
    }
    all_levels.push_back(nsteps);

    // Find the angle that is closest to the local target direction and store it in closest_dir_index
    if (std::fabs(angle - tgt_dir) < min_dir_diff) {
      cloest_dir_index = all_levels.size() - 1;
      min_dir_diff = std::fabs(angle - tgt_dir);
    }

    // First assignment to last_dist_level that stores the distance level (nsteps) for the last checked angle.
    if (last_dist_level < 0) {
      last_dist_level = nsteps;
      continue;
    }

    if (sn_start) {
      // If the distance at the current angle is at least one level closer than the one at the last angle, or the
      // distance at the last angle is out of range but the distance at the current angle is less than it, then the end
      // point of a safezone is found.
      if (nsteps <= (last_dist_level - safezone_det_thresh_) ||
          (last_dist_level >= number_of_steps_ && nsteps < last_dist_level)) {
        if (std::fabs(nsteps * range_step_size_ + deadzone_radius_ - start_point.dist) < safezone_max_width_) {
          sn_start = false;
          end_point.dist = nsteps * range_step_size_ + deadzone_radius_;
          end_point.angle = angle - angle_step_;
          llevel = nsteps;
          safezone.setSimpleSafeZone(end_point.dist, end_point.angle, start_point.dist, start_point.angle, llevel,
                                     rlevel, neighbor_search_diameter_);
          safezones_.push_back(safezone);
          // last_safezone_leftbound = end_point.dist;
          last_safezone_leftbound = end_point.angle;
        } else if (start_point.dist > (nsteps * range_step_size_ + deadzone_radius_)) {
          // This start point is useless
          sn_start = false;
        }
      } else if (nsteps > last_dist_level) {
        // If the distance level at the current angle is greater than the one at the last angle, use the distance level
        // at the last angle to calculate the distance of the start point.
        start_point.dist = last_dist_level * range_step_size_ + deadzone_radius_;
        start_point.angle = angle;
        rlevel = nsteps;
      }
    } else {  // !sn_start
      // If the distance level at the current angle is at least one level farther than the one at the last angle, or no
      // obstacle found at the current angle, then take the distance at the last angle as the start point of a new
      // safezone, while keeping the current angle and distance level.
      if (nsteps >= (last_dist_level + safezone_det_thresh_) || !has_obst) {
        sn_start = true;
        start_point.dist = last_dist_level + range_step_size_ + deadzone_radius_;
        start_point.angle = angle;
        rlevel = nsteps;
      } else if (nsteps <= (last_dist_level - safezone_det_thresh_)) {  // less safer?
        end_point.dist = nsteps * range_step_size_ + deadzone_radius_;
        end_point.angle = angle - angle_step_;
        llevel = nsteps;

        // find start point backwards
        int tmp_level = last_dist_level;
        int last_level = nsteps;
        int cnt = all_levels.size() - 2;
        while (std::fabs(end_point.dist - tmp_level * range_step_size_ + deadzone_radius_) > safezone_max_width_ ||
               tmp_level >= (last_level - safezone_det_thresh_)) {
          last_level = tmp_level;
          tmp_level = all_levels[cnt];
          if (--cnt < 0)
            break;
        }

        start_point.dist = tmp_level * range_step_size_ + deadzone_radius_;
        start_point.angle = angle_step_ * (cnt + 1) - half_fov_;
        if (start_point.angle > last_safezone_leftbound) {
          rlevel = tmp_level;
          safezone.setSimpleSafeZone(end_point.dist, end_point.angle, start_point.dist, start_point.angle, llevel,
                                     rlevel, neighbor_search_diameter_);
          safezones_.push_back(safezone);
          last_safezone_leftbound = end_point.angle;
        } else {
          double dist_diff_1 = std::fabs(start_point.dist - end_point.dist);
          double dist_diff_2 = std::fabs(safezones_[safezones_.size() - 1].right.dist - 
                                         safezones_[safezones_.size() - 1].left.dist);
          if (dist_diff_1 < dist_diff_2) {  // replace the last safezone
            safezones_[safezones_.size() - 1].setSimpleSafeZone(end_point.dist, end_point.angle, start_point.dist,
                                                                start_point.angle, llevel, rlevel, neighbor_search_diameter_);
            last_safezone_leftbound = end_point.angle;
          }
        }
      }
    }
    last_dist_level = nsteps;
  }

  if (sn_start && last_dist_level >= number_of_steps_) {                      // out of range
    end_point.dist = number_of_steps_ * range_step_size_ + deadzone_radius_;  // maximum obstacle dist
    end_point.angle = angle - angle_step_;
    llevel = nsteps;
    safezone.setSimpleSafeZone(end_point.dist, end_point.angle, start_point.dist, start_point.angle, llevel, rlevel,
                               neighbor_search_diameter_);
    safezones_.push_back(safezone);
  }

  pcl::PointXYZ self(0, 0, 0);
  // pcl::PointXYZ local_tgt_pose(local_tgt_pose_.pose.position.x, local_tgt_pose_.pose.position.y, local_tgt_pose_.pose.position.z);
  if (lidar_xyz_oct_.isBlock<unibn::L2Distance<pcl::PointXYZ>>(local_tgt_pclxyz_, self, 0.6)) {
    tgt_point_blocked_ = 1;
    // this sets the oa direction nearly to center line of the robot body
    safezone.setSimpleSafeZone(-1, 0, -1, 0, 0, 0, range_step_size_);
    safezones_.insert(safezones_.begin(), safezone);
  } else {
    tgt_point_blocked_ = -1;
    int level = all_levels[cloest_dir_index];
    int next_level = level;
    int step = 0;
    if (level >= 1) {
      while (next_level == level && step < 15 && (cloest_dir_index - step) > 0) {
        next_level = all_levels[cloest_dir_index - step];
        ++step;
      }

      if (level <= next_level) 
        next_level = level - 1;

      start_point.dist = (0.5 + next_level) * range_step_size_;
      start_point.angle = angle_step_ * (cloest_dir_index - step + 1) - half_fov_;
      llevel = level - 1;
      step = 0;
      next_level = level;

      while (next_level == level && step < 15 && (cloest_dir_index + step) < all_levels.size()) {
        next_level = all_levels[cloest_dir_index + step];
        ++step;
      }

      if (level <= next_level) 
        next_level = level - 1;

      end_point.dist = (0.5 + next_level) * range_step_size_;
      end_point.angle = angle_step_ * (cloest_dir_index + step - 1) - half_fov_;
      rlevel = level - 1;
      safezone.setSimpleSafeZone(end_point.dist, end_point.angle, start_point.dist, start_point.angle, llevel, rlevel,
                                 neighbor_search_diameter_);
      safezones_.insert(safezones_.begin(), safezone);
    }
  }
}

int ObstacleAvoid::findNearestSafeZone(float orig_dir, float tgt_dist, float& oa_dir_diff, float& oa_dist,
                                       float& oa_dir, float& body_dir_diff, bool& in_safezone) {
  if (safezones_.empty()) {
    return -1;
  }

  in_safezone = false;
  int nearest_index = -1;
  const size_t num_of_safezone = safezones_.size();
  float min_dtheta = 360.0;
  float dtheta, dist, dir;
  float widths[num_of_safezone], left_xs[num_of_safezone], left_ys[num_of_safezone], right_xs[num_of_safezone],
        right_ys[num_of_safezone];
  float max_width = 0.0;
  bool is_passable;

  for (size_t i = 0; i < safezones_.size(); ++i) {
    left_xs[i] = safezones_[i].left.dist * std::cos(safezones_[i].left.angle);
    left_ys[i] = safezones_[i].left.dist * std::sin(safezones_[i].left.angle);
    right_xs[i] = safezones_[i].right.dist * std::cos(safezones_[i].right.angle);
    right_ys[i] = safezones_[i].right.dist * std::sin(safezones_[i].right.angle);
    widths[i] = sqrt(pow(left_xs[i] - right_xs[i], 2) + pow(left_ys[i] - right_ys[i], 2));

    if (widths[i] >= (robot_width_ + robot_safe_gap_)) {  // with safe gap
      is_passable = true;
      dtheta = safezones_[i].getDirectionDiff(orig_dir, tgt_dist, dir, dist);
    } else if (widths[i] >= max_width) {
      max_width = widths[i];
      if (max_width > robot_width_) {  // without safe gap
        is_passable = true;
        dtheta = safezones_[i].getDirectionDiff(orig_dir, tgt_dist, dir, dist);
      } else {
        is_passable = false;
        max_width = 0.0;
      }
    }

    if (dtheta < -7) 
      continue;

    // Nearest safezone: passable and dtheta is minimum
    if ((std::fabs(dtheta) < std::fabs(min_dtheta)) && is_passable) {
      min_dtheta = dtheta;
      oa_dist = dist;
      oa_dir = dir;
      nearest_index = i;
      body_dir_diff = dir;
      if (safezones_[i].left_dir > 0 && safezones_[i].right_dir < 0) {
        in_safezone = true;
      } else {
        in_safezone = false;
      }
    }
  }

  oa_dir_diff = min_dtheta;
  mod_oa_dir_ = oa_dir;
  showNearestSafezone(orig_dir, nearest_index);

  return nearest_index;
}

void ObstacleAvoid::showNearestSafezone(float orig_dir, int nearest_index) {
  int imglx, imgly, imgrx, imgry;
  if (is_view_safezone_lidar_) {
    imglx = kImgCol / 2 - static_cast<int>(safezones_[nearest_index].left.dist * sin(safezones_[nearest_index].left.angle) * kImgRow / 5);
    imgly = kImgRow - 1 - static_cast<int>(safezones_[nearest_index].left.dist * cos(safezones_[nearest_index].left.angle) * kImgRow / 5);
    imgrx = kImgCol / 2 - static_cast<int>(safezones_[nearest_index].right.dist * sin(safezones_[nearest_index].right.angle) * kImgRow / 5);
    imgry = kImgRow - 1 - static_cast<int>(safezones_[nearest_index].right.dist * cos(safezones_[nearest_index].right.angle) * kImgRow / 5);
    cv::line(safezone_view_, cv::Point(imglx, imgly), cv::Point(imgrx, imgry), cv::Scalar(0, 0, 255), 4);

    // Draw original direction
    imglx = kImgCol / 2 - static_cast<int>(3 * sin(orig_dir) * kImgRow / 5);
    imgly = kImgRow - 1 - static_cast<int>(3 * cos(orig_dir) * kImgRow / 5);
    imgrx = kImgCol / 2;
    imgry = kImgRow - 1;
    cv::line(safezone_view_, cv::Point(imglx, imgly), cv::Point(imgrx, imgry), cv::Scalar(255, 0, 0), 1);

    // Draw chosen direction
    imglx = kImgCol / 2 - static_cast<int>(3 * sin(mod_oa_dir_) * kImgRow / 5);
    imgly = kImgRow - 1 - static_cast<int>(3 * cos(mod_oa_dir_) * kImgRow / 5);
    imgrx = kImgCol / 2;
    imgry = kImgRow - 1;
    cv::line(safezone_view_, cv::Point(imglx, imgly), cv::Point(imgrx, imgry), cv::Scalar(123, 211, 213), 3);

    char temp[21];
    sprintf(temp, "FPS: %05f", 1000.0 / time_intvl2_);
    cv::putText(safezone_view_, temp, cv::Point(0, 40), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(1, 0, 0, 0));
  }
}

void ObstacleAvoid::smoothVelocity() {
  float max_dvx = 0.02;
  float max_drz = 0.03;

  if (oa_vel_vx_ - last_vx_ > max_dvx)
    oa_vel_vx_ = last_vx_ + max_dvx;
  else if (oa_vel_vx_ - last_vx_ < -max_dvx)
    oa_vel_vx_ = last_vx_ - max_dvx;
  last_vx_ = oa_vel_vx_;

  if (oa_vel_rz_ - last_rz_ > max_drz)
    oa_vel_rz_ = last_rz_ + max_drz;
  else if (oa_vel_rz_ - last_rz_ < -max_drz)
    oa_vel_rz_ = last_rz_ - max_drz;
  last_rz_ = oa_vel_rz_;
}

void ObstacleAvoid::lidarCallback(const sensor_msgs::LaserScanConstPtr& msg) { 
  int64_t tc_begin = 0, tc_end = 0;
  tc_begin = cv::getTickCount();
  findSafeZonesWithOctree(msg); 
  tc_end = cv::getTickCount();
  time_intvl2_ = 1000 * double(tc_begin - tc_end) / cv::getTickFrequency();
}

void ObstacleAvoid::readParameters() {
  ros::NodeHandle nh("~");
  nh.getParam("use_lidar", lidar_used_);
  // nh.param("use_lidar", lidar_used_, true);
  nh.param("view_safezone_lidar", is_view_safezone_lidar_, true);
  nh.param("robot_width", robot_width_, 0.15);
  nh.param("robot_safe_gap", robot_safe_gap_, 0.2);
  nh.param("max_vx", max_vx_, 0.5);
  nh.param("max_rz", max_rz_, M_PI_2);
  nh.param("min_turn_radius", min_turn_radius_, 0.0);
  nh.param("safezone_rest_wide", safezone_max_width_, 0.5);
  nh.param("largest_err_direction", max_direction_diff_, 1.57);
  nh.param("time_of_halfpi", valid_half_pi_, 1.0);
  nh.param("safezone_lvl_err_threshold", safezone_det_thresh_, 1.0);
  nh.param("lvl_absolute_safe", abs_safe_dist_, 1.0);
  nh.param("a_theta", a_theta_, 0.5);
  nh.param("b_alpha", b_alpha_, 0.1);
  nh.param("c_dist", c_dist_, 0.25);
  nh.param("det_tolerance", obst_det_thresh_, 2.0);
  nh.param("lidar_x", tr_from_base_to_lidar_[0], 0.0);
  nh.param("lidar_y", tr_from_base_to_lidar_[1], 0.0);
  nh.param("lidar_z", tr_from_base_to_lidar_[2], 0.0);
  nh.param("half_fov", half_fov_, 0.52);

  if (lidar_used_) {
    max_valid_half_fov_ = valid_half_pi_ * M_PI_2;
    nh.param("obs_times_of_deld", number_of_steps_, 4.0);
    nh.param("obs_dangle", angle_step_, 0.058);
    nh.param("deld", neighbor_search_diameter_, 0.5);
    nh.param("deadzone", deadzone_radius_, 0.2);
    nh.param("range_step_size", range_step_size_, 0.2);
  }
}

void ObstacleAvoid::initPublishesAndSubscribes() {
  ros::NodeHandle nh;
  if (lidar_used_) 
    lidar_sub_ = nh.subscribe("/scan", 1, &ObstacleAvoid::lidarCallback, this);
}

void ObstacleAvoid::safeZoneView() {
  int imglx, imgly, imgrx, imgry;
  std_msgs::Float32MultiArray msg;
  
  for (auto& elem : safezones_) {
    msg.data.push_back(elem.left.dist);
    msg.data.push_back(elem.left.angle);
    msg.data.push_back(elem.right.dist);
    msg.data.push_back(elem.right.angle);

    if (is_view_safezone_lidar_) {
      imglx = kImgCol / 2 - static_cast<int>(elem.left.dist * sin(elem.left.angle) * kImgRow / 5);
      imgly = kImgRow - 1 - static_cast<int>(elem.left.dist * cos(elem.left.angle) * kImgRow / 5);
      imgrx = kImgCol / 2 - static_cast<int>(elem.right.dist * sin(elem.right.angle) * kImgRow / 5);
      imgry = kImgRow - 1 - static_cast<int>(elem.right.dist * cos(elem.right.angle) * kImgRow / 5);
      cv::line(safezone_view_, cv::Point(imglx, imgly), cv::Point(imgrx, imgry), cv::Scalar(0, 255, 0), 1);
    }
  }

  if (is_view_safezone_lidar_) {
    cv::imshow("ObstacleView", safezone_view_);
    cv::waitKey(1);
  }
}

}
