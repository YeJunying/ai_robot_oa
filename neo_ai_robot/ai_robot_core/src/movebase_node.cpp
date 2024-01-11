#include "movebase.h"

#include <tf2_ros/transform_listener.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "movebase_node");
  tf2_ros::Buffer tf(ros::Duration(10));
  tf2_ros::TransformListener tfl(tf);

  ai_robot::MoveBase movebase(tf);

  ros::spin();

  return 0;
}
