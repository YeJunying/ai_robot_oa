#ifndef AI_ROBOT_MOVEBASE_CLIENT_H
#define AI_ROBOT_MOVEBASE_CLIENT_H

#include <Eigen/Dense>

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <actionlib/client/simple_action_client.h>
#include <ai_robot_msgs/MoveBaseAction.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PoseStamped.h>

namespace ai_robot {

typedef actionlib::SimpleActionClient<ai_robot_msgs::MoveBaseAction> MoveBaseActionClient;

enum class MoveBaseTaskState {
  kNotStarted,
  kActive,
  kDone,
  kCancelled
};

class MoveBaseClient {
public:
  MoveBaseClient();
  virtual ~MoveBaseClient();

public:
  void doTask(const geometry_msgs::PoseStamped& goal);
  void cancelTask();
  // void doTask(double src_x, double src_y, int src_lvl, double dst_x, double dst_y, int dst_lvl);
  void doneCb(const actionlib::SimpleClientGoalState& state, const ai_robot_msgs::MoveBaseResultConstPtr& result);
  void feedbackCb(const ai_robot_msgs::MoveBaseFeedbackConstPtr& feedback);
  void activeCb();
  MoveBaseTaskState getState() const { return state_; }

private:
  void initialPoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
  void manctlStateCb(const std_msgs::UInt8ConstPtr& msg);
  // void getMapInfo();

private:
  ros::NodeHandle nh_;
  MoveBaseActionClient *ac_;
  MoveBaseTaskState state_;
  size_t from_waypoint, to_waypoint;
  ros::Subscriber manctl_state_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber initpose_sub_;
  Eigen::Vector2d start_pos_;
};

}

#endif // !AI_ROBOT_MOVEBASE_CLIENT_H
