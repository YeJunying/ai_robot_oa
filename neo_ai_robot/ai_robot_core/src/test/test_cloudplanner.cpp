#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "../cloudplanner.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "test_cloudplanner");

  std::string url = "ws://139.196.197.176:30110";

  if (argc == 2)
    url = argv[1];

  // ai_robot::CloudPlanner planner(url);
  ai_robot::CloudPlanner planner(1.0);

  std::string request =
   "{"
      "\"points\": ["
      "{"
          "\"x\": 13519977.939568073,"
          "\"y\": 3614361.787713769,"
          "\"level\": 1"
      "},"
      "{"
          "\"x\": 13519955.121249475,"
          "\"y\": 3614354.0630268487,"
          "\"level\": 1"
      "}"
    "]"
   "}";

  ros::Time ts = ros::Time::now();
  geometry_msgs::PoseStamped start, end;
  start.header.stamp = ts;
  start.pose.position.x = 13519977.939568073;
  start.pose.position.y = 3614361.787713769;
  end.header.stamp = ts;
  end.pose.position.x = 13519955.121249475;
  end.pose.position.y = 3614354.0630268487;
  std::vector<geometry_msgs::PoseStamped> plan;

  if (planner.makePlan(start, end, plan)) {
    for (auto p : plan) {
      std::cout << "x=" << std::fixed << std::setprecision(10) << p.pose.position.x << ",y=" << std::fixed << std::setprecision(10) << p.pose.position.y << '\n';
    }
  }

  planner.close();
}
