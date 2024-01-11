#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "movebase_client.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "movebase_client");
  ai_robot::MoveBaseClient client;

  geometry_msgs::PoseStamped goal;
  goal.header.stamp = ros::Time::now();
  goal.pose.position.x = 13519951.951173093;
  goal.pose.position.y = 3614357.100373561;
  client.doTask(goal);

  ros::spin();

  return 0;
}
