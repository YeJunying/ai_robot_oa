#include "cloudplanner.h"

#include <functional>
#include <iostream>
#include <mutex>
#include <vector>
#include <fstream>

#include <ros/ros.h>
#include <ros/console.h>

#include <rapidjson/document.h>
#include "bluetooth.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/GetMapRequest.h"
#include "rapidjson/reader.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

#include "bluetooth.h"

namespace ai_robot {

namespace {
// NOTE:: The X-axis and Y-axis of the plane image do not have the same aspect ratio.
// const double kAspectRatioY2X = 0.07826 / 0.0629;
const double kAspectRatioY2X = 1.1;
}

const std::string CloudPlanner::req_json_tpl_ =
  "{"
    "\"points\": ["
      "{"
        "\"x\": null,"
        "\"y\": null,"
        "\"level\": null"
      "},"
      "{"
          "\"x\": null,"
          "\"y\": null,"
          "\"level\": null"
      "}"
    "]"
  "}";

CloudPlanner::CloudPlanner(const std::string& url, float scale)
    : url_(url),
      is_connected_(false),
      message_parsed_(false),
      scale_(scale) {
  client_.setCustomMessageHandler(std::bind(&CloudPlanner::parseMessage, this, std::placeholders::_1));
  doc_.Parse<rapidjson::kParseFullPrecisionFlag>(req_json_tpl_.c_str());
  connect(url);
}

CloudPlanner::CloudPlanner(float scale)
    : is_connected_(false),
      message_parsed_(false),
      scale_(scale) {
  client_.setCustomMessageHandler(std::bind(&CloudPlanner::parseMessage, this, std::placeholders::_1));
  doc_.Parse<rapidjson::kParseFullPrecisionFlag>(req_json_tpl_.c_str());
  
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.getParam("cloud_planner_url", url_);
  if (!url_.empty()) {
    ROS_INFO_NAMED("cloud_planner", "cloud url: %s", url_.c_str());
    connect(url_);
  } else {
    ROS_INFO_NAMED("cloud_planner", "failed to get url");
  }
}

CloudPlanner::~CloudPlanner() {
}

bool CloudPlanner::connect(const std::string url) {
  is_connected_ = client_.connect(url);
  return is_connected_;
}

bool CloudPlanner::request(double src_x, double src_y, int src_lvl, double dst_x, double dst_y, int dst_lvl) {
  doc_["points"][0]["x"] = src_x;
  doc_["points"][0]["y"] = src_y;
  doc_["points"][0]["level"] = src_lvl;
  doc_["points"][1]["x"] = dst_x;
  doc_["points"][1]["y"] = dst_y;
  doc_["points"][1]["level"] = dst_lvl;
  auto req = stringify(doc_);
  ROS_INFO_NAMED("cloud_planner", "Sending request: start(%.10f,%.10f), end(%.10f,%.10f)", src_x, src_y, dst_x, dst_y);
  return client_.sendTextDataTillSuccess(req);
}

std::string CloudPlanner::stringify(const rapidjson::Document& doc) {
  static rapidjson::StringBuffer strbuf;
  static rapidjson::Writer<rapidjson::StringBuffer> writer(strbuf);
  strbuf.Clear();
  writer.Reset(strbuf);
  doc.Accept(writer);
  return strbuf.GetString();
}

void CloudPlanner::parseMessage(const std::string& msg) {
  static rapidjson::Document doc;
  message_parsed_ = false;

  doc.Parse<rapidjson::kParseFullPrecisionFlag>(msg.c_str());
  if (doc.HasParseError())
    return;

  rapidjson::Value::ConstValueIterator it; 
  waypoints_.clear();
  for (it = doc.Begin(); it != doc.End(); ++it) {
    waypoints_.push_back(bluetooth::Waypoint(it->GetObject()["x"].GetDouble(), it->GetObject()["y"].GetDouble(), it->GetObject()["level"].GetInt()));
  }
  
  message_parsed_ = true;
}

bool CloudPlanner::close() {
  return client_.close();
}

// @start: point in map relative to left-bottom
// @goal: point in map relative to left-bottom
// @plan: a vector of waypoints using mercator cooridnates
bool CloudPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                            const geometry_msgs::PoseStamped& goal,
                            std::vector<geometry_msgs::PoseStamped>& plan) {
  // return makePlanFromFile(start, goal, plan);
  return makeDummyPlan(start, goal, plan);

  if (!is_connected_) {
    ROS_ERROR_NAMED("cloud_planner", "Server is not connected!");
    return false;
  }

  // ros::NodeHandle nh;
  double src_x, src_y, dst_x, dst_y;
  std::cout << "scale: " << scale_ << '\n';
  ROS_INFO("src: (%f, %f), dst: (%f, %f)", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
  bluetooth::Transform::plane2merca(start.pose.position.x, start.pose.position.y, src_x, src_y, scale_);
  bluetooth::Transform::plane2merca(goal.pose.position.x, goal.pose.position.y, dst_x, dst_y, scale_);
  ROS_INFO("src: (%f, %f), dst: (%f, %f)", src_x, src_y, dst_x, dst_y);

  if (!request(src_x, src_y, 1, dst_x, dst_y, 1)) {
    ROS_ERROR_NAMED("cloud_planner", "Failed to get response");
    return false;
  }

  // geometry_msgs::PoseStamped wp_to_map;
  // wp_to_map.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped wp_pose;
  wp_pose.header.stamp = ros::Time::now();
  double x, y;
  for (auto it = ++waypoints_.begin(); it != waypoints_.end(); ++it) {
  // for (const auto wp : waypoints_)
    // Convert waypoint position from mercator to 2d-plane
    // x = static_cast<double>(static_cast<long int>(wp.x));
    // y = static_cast<double>(static_cast<long int>(wp.y));
    // bluetooth::Transform::merca2plane(wp.x, wp.y, wp_to_map.pose.position.x, wp_to_map.pose.position.y, scale_);
    // bluetooth::Transform::merca2plane(x, y, wp_to_map.pose.position.x, wp_to_map.pose.position.y, scale_);
    // wp_to_map.pose.position.x *= kAspectRatioY2X;
    // plan.push_back(wp_to_map);
    wp_pose.pose.position.x = it->x;
    wp_pose.pose.position.y = it->y;
    plan.push_back(wp_pose);
  }

  return true;
}

bool CloudPlanner::makePlanFromFile(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
  std::cout << "Make dummpy plan\n";
  const std::string filename = "/home/jerryx/path.txt";
  std::ifstream infile(filename);

  if (!infile.is_open()) {
    std::cerr << "Error opening file " << filename << '\n';
    return false;
  }

  double x, y;
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  while (infile >> x >> y) {
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    plan.push_back(pose);
  }

  return true;
}

bool CloudPlanner::makeDummyPlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
  // plan.push_back(start);
  plan.push_back(goal);
  return true;
}

}
