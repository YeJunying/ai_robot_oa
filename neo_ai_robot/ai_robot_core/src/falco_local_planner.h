//
// Created by nhc on 23-12-24.
//

#ifndef SRC_FALCO_LOCAL_PLANNER_H
#define SRC_FALCO_LOCAL_PLANNER_H

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
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/UInt8.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/exceptions.h>

#include "ai_robot_msgs/OAVelocity.h"
#include "ai_robot_msgs/SetRobotPose.h"
#include "ai_robot_msgs/SetTargetPosition.h"
#include "mapinfo.h"

#include "util.h"

namespace ai_robot {
    class FalcoLocalPlanner {
    public:
        FalcoLocalPlanner(MapInfo *mapinfo);
        virtual ~FalcoLocalPlanner();

    public:
        void initialize(std::string name, tf2_ros::Buffer *tf);
        bool isInitialized() { return initialized_; }
        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
        bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
        bool isGoalReached();
        bool isLockedByMan() { return locked_by_man_; }

    private:
        void velocityCB(const geometry_msgs::TwistStamped::ConstPtr& msg);
        void manctlStateCB(const std_msgs::UInt8ConstPtr& msg);
        void Laserscan2DCB(const sensor_msgs::LaserScan::ConstPtr &msg);

    private:
        bool initialized_;
        bool locked_by_man_;
//        bool imu_has_mag_;
        // Eigen::Vector2d cur_pos_;
        // geometry_msgs::PoseStamped cur_pose_to_origin_;
        // geometry_msgs::PoseStamped dst_pose_to_origin_;
        // geometry_msgs::PoseStamped cur_waypoint_to_origin_;

        // Rotation matrix from odom to earth (mercator)
        // Eigen::Matrix2d rotm_odom2merca_;
        // Eigen::Vector2d tr_odom2merca_;

        // Pose
//        double initial_hdg_;
//        double hdg_;
//        double new_hdg_;
//        Eigen::Vector2d pos_;
//        Eigen::Vector2d blue_pos_;
//        Eigen::Vector2d waypoint_in_merca_;
        geometry_msgs::PoseStamped odom_pose_;

        // pose_t initial_pose_;
        // pose_t cur_pose_;

        // Constraints/Limits
//        double max_vel_x_;
//        double max_vel_y_;

        // Plan
//        bool has_new_plan_;
        std::vector<geometry_msgs::PoseStamped> global_plan_;
        // std::vector<geometry_msgs::PoseStamped> global_plan_map_;
        // std::vector<geometry_msgs::PoseStamped> global_plan_odom_;
        // std::vector<geometry_msgs::PoseStamped> local_plan_;
        // std::vector<geometry_msgs::PoseStamped> waypoints_in_odom_;
//        size_t from_index_, to_index_;
//        double odom_dist_to_waypoint_;
//        double blue_dist_to_waypoint_;
//        double next_dist_from_odom_;
        bool waypoint_reached_;
//        bool turn_done_;
//        bool new_hdg_updated_;
        bool goal_reached_;

        // publishers/subscribers
        ros::Subscriber oa_vel_sub_;
        ros::Subscriber manctl_state_sub_;
        ros::Subscriber scan_sub_;
        ros::Publisher registered_scan_pub_;
        ros::Publisher waypoint_pub_;
        geometry_msgs::TwistStamped cmd_vel_;
        geometry_msgs::PointStamped waypointMsgs_;
        tf2_ros::Buffer *tf_;
        tf2_ros::TransformListener *tfl_;

//        MapInfo *mapinfo_;
        // ros::ServiceClient mapinfo_srv_;

//        double defl_from_neu_to_map_; // deflection from ENU frame (mercator) to map frame
//        double defl_from_nwu_to_map_; // deflection from NWU frame (magnetometer) to map frame
//        bool is_merca_aligned_to_map_; // whether mercator frame is rotated to align with map frame

        // for communication with oa python node
        double oa_vel_vx_;
        double oa_vel_rz_;
//        double guide_angle_;
//        ros::ServiceClient oa_pose_srv_, oa_tgt_srv_;

//        double transform_tolerance_;
    };
}

#endif //SRC_FALCO_LOCAL_PLANNER_H
