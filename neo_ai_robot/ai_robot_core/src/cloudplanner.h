#ifndef AI_ROBOT_CLOUD_PLANNER
#define AI_ROBOT_CLOUD_PLANNER

#include <iostream>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>

#include "rapidjson/document.h"

#include "bluetooth.h"
#include "websocket/client.h"

namespace ai_robot {

// A global path planner that fetches result from clould service via websock.
class CloudPlanner {
public:
  CloudPlanner(float resolution);
  CloudPlanner(const std::string& url, float resolution);
  virtual ~CloudPlanner();

public:
  bool connect(const std::string url);
  bool close();
  bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
  const std::vector<bluetooth::Waypoint>& getPath() { return waypoints_; }

private:
  std::string stringify(const rapidjson::Document& doc);
  bool request(double src_x, double src_y, int src_lvl, double dst_x, double dst_y, int dst_lvl);
  void parseMessage(const std::string& msg);
  bool makePlanFromFile(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
  bool makeDummyPlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

// public:
//   std::vector<blue::Waypoint> waypoints;

private:
  std::string url_;
  static const std::string req_json_tpl_;
  rapidjson::Document doc_;
  WebSocketClient client_;
  bool is_connected_;
  bool message_parsed_;
  float scale_;
  std::vector<bluetooth::Waypoint> waypoints_;
};

}

#endif // !AI_ROBOT_GLOBAL_PLANNER
