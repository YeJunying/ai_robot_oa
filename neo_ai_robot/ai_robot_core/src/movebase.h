#ifndef AI_ROBOT_MOVEBASE_ACTION_H
#define AI_ROBOT_MOVEBASE_ACTION_H

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/buffer.h>

#include "ai_robot_msgs/MoveBaseAction.h"

#include "cloudplanner.h"
// #include "simple_local_planner.h"
//#include "obstacle_avoid.h"
#include "mapinfo.h"
#include "falco_local_planner.h"

namespace ai_robot {

typedef actionlib::SimpleActionServer<ai_robot_msgs::MoveBaseAction> MoveBaseActionServer;

enum MoveBaseState {
  kPlanning,
  kControlling,
  kClearing
};

enum RecoveryTrigger {
  kPlanningR,
  kControllingR,
  kOscillationR,
};

class MoveBase {
public:
  MoveBase(tf2_ros::Buffer& tf);
  virtual ~MoveBase();

  bool executeCycle(geometry_msgs::PoseStamped& goal);

private:
  // bool planService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {

  void transformPositionFromMercatorToMap(double in_x, double in_y, double& out_x, double& out_y);
  bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
  void publishVelocity(const geometry_msgs::Twist& cmd_vel);
  void publishZeroVelocity();
  void resetState();
  void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);
  void planThread();
  void executeCb(const ai_robot_msgs::MoveBaseGoalConstPtr& movebase_goal);
  bool getRobotPose(geometry_msgs::PoseStamped& global_pose); // bluetooth localization
  double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);
  geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);
  void wakePlanner(const ros::TimerEvent& event);
  // void reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level);
  void getMapInfo();

private:
  tf2_ros::Buffer& tf_;
  MoveBaseActionServer *as_;
  // NOTE: In the future, both global and local planner need to replaced with abstract base class and use pluginlib to
  // load subclasses that implement the require interfaces.
  std::shared_ptr<CloudPlanner> planner_;
  // std::shared_ptr<SimpleLocalPlanner> tc_;
//  std::shared_ptr<ObstacleAvoid> tc_;
  std::shared_ptr<FalcoLocalPlanner> tc_;
  std::string robot_base_frame, global_frame;

  unsigned int recovery_index_;

  geometry_msgs::PoseStamped global_pose_;
  double planner_freq_, controller_freq_;
  double planner_patience_, controller_patience_;
  int32_t max_planning_retries_;
  uint32_t planning_retries_;
  ros::Publisher vel_pub_, action_goal_pub_, current_goal_pub_, recovery_status_pub_;
  ros::Subscriber goal_sub_;
  // ros::ServiceServer makeplan_srv_;
  double oscillation_timeout_, oscillation_distance_;

  MoveBaseState state_;
  RecoveryTrigger recovery_trigger_;

  ros::Time t_last_valid_plan_, t_last_valid_control_, t_last_oscillation_reset_;
  geometry_msgs::PoseStamped oscillation_pose_;

  // set up plan triple buffer
  std::vector<geometry_msgs::PoseStamped> *planner_plan_;
  std::vector<geometry_msgs::PoseStamped> *latest_plan_;
  std::vector<geometry_msgs::PoseStamped> *controller_plan_;

  bool run_planner_;
  boost::recursive_mutex planner_mutex_;
  boost::condition_variable_any planner_cond_;
  geometry_msgs::PoseStamped planner_goal_;
  boost::thread *planner_thread_;

  boost::recursive_mutex configuration_mutex_;
  // dynamic_reconfigure::Server<ai_robot::MoveBaseConfig> *dsrv_;

  bool setup_, p_freq_change_, c_freq_change_;
  bool has_new_global_plan_;

  double transform_tolerance_;

  MapInfo mapinfo_;
  ros::ServiceClient mapinfo_srv_;
};

}


#endif // !AI_ROBOT_MOVE_BASE_ACTION_H
