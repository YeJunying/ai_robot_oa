#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "ros/init.h"
#include <cmath>
#include <string>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
// #include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

const std::string kGlobalFrameId("map");
const std::string kBaseFrameId("base_link");
const std::string kOdomFrameId("odom");

tf2_ros::Buffer *tf;
tf2_ros::TransformListener *tfl;
tf2_ros::TransformBroadcaster *tfb;
geometry_msgs::TransformStamped tf_stamped;
bool initial_pose_updated = false;

typedef struct {
  // v[0]: x, v[1]: y, v[2]: yaw
  double v[3] = {0};
} pose_vector_t;

pose_vector_t getPoseVectorZero() {
  pose_vector_t c;
  c.v[0] = 0.0;
  c.v[1] = 0.0;
  c.v[2] = 0.0;
  return c;
}

void setPoseVector(pose_vector_t *c, double x, double y, double yaw) {
  c->v[0] = x;
  c->v[1] = x;
  c->v[2] = yaw;
}

static std::string stripLeadingSlash(const std::string& in) {
  std::string out = in;
  if (!in.empty() && in[0] == '/')
    out.erase(0, 1);
  return out;
}

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
  // static pose_vector_t last_pose_v;
  static tf2::Transform latest_tf;
  // static bool latest_tf_valid = false;
  // bool sent_first_transform;

  if (msg.header.frame_id == "") {
    ROS_WARN("Received initial pose with empty frame_id. You should always supply a frame_id.");
  } else if (stripLeadingSlash(msg.header.frame_id) != kGlobalFrameId) {
    ROS_WARN("Ignoring initial pose in frame \"%s\"; initial poses must be be in the global frame, \"%s\"",
             stripLeadingSlash(msg.header.frame_id).c_str(),
             kGlobalFrameId.c_str());
    return;
  }
  geometry_msgs::PoseStamped odom_to_map; // map origin in odom frame
  try {
    tf2::Quaternion q;
    tf2::convert(msg.pose.pose.orientation, q);
    tf2::Transform tmp_tf(q, tf2::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, 0.0));

    geometry_msgs::PoseStamped tmp_tf_stamped; // map origin in base frame
    tmp_tf_stamped.header.frame_id = kBaseFrameId;
    tmp_tf_stamped.header.stamp = msg.header.stamp;
    tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);

    tf->transform(tmp_tf_stamped, odom_to_map, kOdomFrameId, ros::Duration(0.1));
  } catch (const tf2::TransformException& ex) {
    ROS_DEBUG("Failed to subtract base to odom transform: %s", ex.what());
    return;
  }
  
  tf2::convert(odom_to_map.pose, latest_tf);
  // latest_tf_valid = true;
  // geometry_msgs::TransformStamped tmp_tf_stamped;
  // tmp_tf_stamped.header.frame_id = kGlobalFrameId;
  // tmp_tf_stamped.header.stamp = ros::Time::now();
  // tmp_tf_stamped.child_frame_id = kOdomFrameId;
  // tf2::convert(latest_tf.inverse(), tmp_tf_stamped.transform);
  // tfb->sendTransform(tmp_tf_stamped);
  // geometry_msgs::TransformStamped tmp_tf_stamped;
  tf_stamped.header.frame_id = kGlobalFrameId;
  tf_stamped.header.stamp = ros::Time::now();
  tf_stamped.child_frame_id = kOdomFrameId;
  tf2::convert(latest_tf.inverse(), tf_stamped.transform);
  initial_pose_updated = true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "odom2map_tf_pub");
  ros::NodeHandle nh;

  tf = new tf2_ros::Buffer();
  tfl = new tf2_ros::TransformListener(*tf);
  tfb = new tf2_ros::TransformBroadcaster();

  // initial_pose_sub = message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>(nh, "/initialpose", 1)
  // initial_pose_filter = tf2_ros::MessageFilter<nav_msgs::Odometry>(initial_pose_sub, tf, kBaseFrameId, 100);
  // initial_pose_filter.registerCallback(initialPoseCallback);
  ros::Subscriber initial_pose_sub = nh.subscribe("/initialpose", 2, initialPoseCallback);

  tf_stamped.header.frame_id = kGlobalFrameId;
  tf_stamped.header.stamp = ros::Time::now();
  tf_stamped.child_frame_id = kOdomFrameId;
  tf_stamped.transform.translation.x = 0;
  tf_stamped.transform.translation.y = 0;
  tf_stamped.transform.translation.z = 0;
  tf_stamped.transform.rotation.x = 0;
  tf_stamped.transform.rotation.y = 0;
  tf_stamped.transform.rotation.z = 0;
  tf_stamped.transform.rotation.w = 1;
  // tfb->sendTransform(tf_stamped);

  ros::Rate r(20);
  while (nh.ok()) {
    if (!initial_pose_updated) {
      tf_stamped.header.stamp = ros::Time::now();
    }
    else {
      initial_pose_updated = false;
    }
    tfb->sendTransform(tf_stamped);
    ros::spinOnce();
    r.sleep();
  }
}
