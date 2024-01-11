//
// Created by nhc on 23-12-24.
//
#include "falco_local_planner.h"

#include <cmath>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include "geometry_msgs/TransformStamped.h"
#include <nav_msgs/GetMap.h>
#include <ros/node_handle.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
//#include <tf.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_ros/transforms.h>

#include "ai_robot_msgs/OAVelocity.h"
#include "ai_robot_msgs/SetRobotPose.h"
#include "ai_robot_msgs/SetTargetPosition.h"

#include "man_control.h"
#include "util.h"

namespace ai_robot {
    FalcoLocalPlanner::FalcoLocalPlanner(MapInfo *mapinfo)
            : initialized_(false),
              locked_by_man_(false),
              goal_reached_(false),
              tf_(nullptr),
              oa_vel_vx_(0.0),
              oa_vel_rz_(0.0) {
    }

    FalcoLocalPlanner::~FalcoLocalPlanner() {
    }

    void FalcoLocalPlanner::initialize(std::string name, tf2_ros::Buffer *tf) {
        if (!isInitialized()) {
            ros::NodeHandle private_nh("~/" + name);
            ros::NodeHandle nh;
            oa_vel_sub_ = nh.subscribe("/pathFollower/cmd_vel", 1, &FalcoLocalPlanner::velocityCB, this);
            manctl_state_sub_ = nh.subscribe("/ai_robot/manctl_state", 1, &FalcoLocalPlanner::manctlStateCB, this);
            scan_sub_ = nh.subscribe("/scan", 5, &FalcoLocalPlanner::Laserscan2DCB, this);

            registered_scan_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/registered_scan",1000);
            waypoint_pub_ = nh.advertise<geometry_msgs::PointStamped> ("/way_point", 5);

            cmd_vel_.header.frame_id = "vehicle";
            waypointMsgs_.header.frame_id = "map";
            goal_reached_ = false;
            waypoint_reached_ = false;

            tf_ = tf;
            tfl_ = new tf2_ros::TransformListener(*tf);
        }
    }

    bool FalcoLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
        global_plan_ = orig_global_plan;

//        waypointMsgs.header.stamp = ros::Time().fromSec(curTime);
        waypointMsgs_.header.stamp = ros::Time().now();
        waypointMsgs_.point.x = global_plan_.front().pose.position.x;
        waypointMsgs_.point.y = global_plan_.front().pose.position.y;
        waypointMsgs_.point.z = global_plan_.front().pose.position.z;
        waypoint_pub_.publish(waypointMsgs_);

        goal_reached_ = false;

        return true;
    }

    bool FalcoLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
        if (locked_by_man_)
            return true;
        cmd_vel.linear.x = oa_vel_vx_;  // centermeter to m
        cmd_vel.angular.z = oa_vel_rz_; // radians to degree
        return true;
    }

    bool FalcoLocalPlanner::isGoalReached() {
//        return goal_reached_;
        return false;
    }

    void FalcoLocalPlanner::velocityCB(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        oa_vel_vx_ = msg->twist.linear.x;
//        oa_vel_rz_ = util::deg2rad(msg->twist.angular.z);  // degrees to radians
        oa_vel_rz_ = msg->twist.angular.z;  // degrees to radians
    }

    void FalcoLocalPlanner::manctlStateCB(const std_msgs::UInt8ConstPtr& msg) {
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

    void FalcoLocalPlanner::Laserscan2DCB(const sensor_msgs::LaserScan::ConstPtr &msg) {
        sensor_msgs::PointCloud2 cloud_msg_2d, cloud_msg_3d_basescan, cloud_msg_3d_map;

        laser_geometry::LaserProjection projector;
        projector.projectLaser(*msg, cloud_msg_2d);

        pcl::PointCloud<pcl::PointXYZI> pclCloud_2d, pclCloud_3d;
        pcl::fromROSMsg(cloud_msg_2d,pclCloud_2d);

        for(int i=0;i<pclCloud_2d.size();i++)  // 将点云z设置为统一值
        {
            pclCloud_2d.points[i].z=0.8;
            pclCloud_2d.points[i].intensity=1.0f;
        }
        pcl::PointCloud<pcl::PointXYZI> cloud_add;
        pcl::copyPointCloud(pclCloud_2d, cloud_add);
        for(int h=0;h<=20;h+=2)
        {
            for(int i=0;i<cloud_add.width;i++)
            {
                cloud_add.points[i].z=(float)h/10;
                cloud_add.points[i].intensity=1.0f;
            }
            pclCloud_3d+=cloud_add;
        }

        pcl::toROSMsg(pclCloud_3d,cloud_msg_3d_basescan);

        // cmu_code need the registration_scan->header.frame_id = "map"
        // here, 2d_scan is "base_scan"
//        geometry_msgs::TransformStamped tfs;

//        cloud_msg_3d_basescan.header.stamp = msg->header.stamp;
//        cloud_msg_3d_basescan.header.frame_id = msg->header.frame_id;

        geometry_msgs::TransformStamped tfs;
//        geometry_msgs::Transform transform;
//        tf::Transform transform;
//        bool lookup_flag = transform.lookupTransform("map", msg->header.frame_id, ros::Time(msg->header.stamp), *tfs);

        try
        {
            tfs = tf_->lookupTransform("map", msg->header.frame_id, ros::Time(msg->header.stamp), ros::Duration(0.1));
//            tfs = tf_->lookupTransform("map", msg->header.frame_id, ros::Time::now(), ros::Duration(0.1));
            pcl_ros::transformPointCloud("map", tfs.transform, cloud_msg_3d_basescan, cloud_msg_3d_map);
            cloud_msg_3d_basescan.header.stamp = msg->header.stamp;
            registered_scan_pub_.publish(cloud_msg_3d_map);
//            registered_scan_pub_.publish(cloud_msg_3d_basescan);
        }
        catch (tf::LookupException &ex)
        {
            ROS_ERROR("%s",ex.what());
//            ros::Duration(1.0).sleep();
        }
//        registered_scan_pub_.publish(cloud_msg_3d_basescan);
    }

}