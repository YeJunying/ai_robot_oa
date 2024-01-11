#include <iostream>

#include <ros/ros.h>

#include "brain.h"
#include "globals.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "brain_node");
  if (argc > 1) {
    ai_robot::g::man_test_mode = true;
  }

  // ros::NodeHandle nh;
  ai_robot::Brain brain;
  ros::Rate r(20);

  while (ros::ok()) {
    // ros::shutdown();
    // brain.testMoveBaseTask();
    brain.think();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}

