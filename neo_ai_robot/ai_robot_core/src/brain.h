#ifndef AI_ROBOT_MAINNODE_H
#define AI_ROBOT_MAINNODE_H

#include <geometry_msgs/PoseStamped.h>

#include "man_control.h"
#include "mapinfo.h"
#include "movebase_client.h"
#include "ros/subscriber.h"

namespace ai_robot {

enum class TaskId {
  kIdle,
  kManControl,
  kMoveBase
};

class Brain {
public:
  // Brain(const ros::NodeHandle& nh);
  Brain();
  virtual ~Brain();

public:
  void think();

private:
  void testMoveBaseTask();
  void doMoveBaseTask(const geometry_msgs::PoseStamped::ConstPtr& goal);
  void getMapInfo();
  void moveBaseGoalCb(const geometry_msgs::PoseStampedConstPtr& msg);

private:
  MoveBaseClient *movebase_client_;
  ManControl *manctl_;
  TaskId taskid_;
  MapInfo mapinfo_;
  ros::NodeHandle nh_;
  ros::Subscriber goal_sub_;
  ros::ServiceClient mapinfo_srv_;
  geometry_msgs::PoseStamped goal_msg_;
  MoveBaseTaskState movebase_task_state_;
};

}

#endif // !AI_ROBOT_MAINNODE_H
