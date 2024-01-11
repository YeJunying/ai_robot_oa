#ifndef AI_ROBOT_LOCAL_PLANNER_LIMITS_H_
#define AI_ROBOT_LOCAL_PLANNER_LIMITS_H_

#include <Eigen/Core>

namespace ai_robot::base_local_planner {

struct LocalPlannerLimits {

  LocalPlannerLimits() {}  

  LocalPlannerLimits(
    double max_vel_trans,
    double min_vel_trans,
    double max_vel_x,
    double min_vel_x,
    double max_vel_y,
    double min_vel_y,
    double max_vel_theta,
    double min_vel_theta,
    double acc_lim_x,
    double acc_lim_y,
    double acc_lim_theta,
    double acc_lim_trans,
    double xy_goal_tolerance,
    double yaw_goal_tolerance,
    double trans_stopped_vel = 0.1,
    double theta_stopped_vel = 0.1)
      : max_vel_trans(max_vel_trans),
        min_vel_trans(min_vel_trans),
        max_vel_x(max_vel_x),
        min_vel_x(min_vel_x),
        max_vel_y(max_vel_y),
        min_vel_y(min_vel_y),
        max_vel_theta(max_vel_theta),
        min_vel_theta(min_vel_theta),
        acc_lim_x(acc_lim_x),
        acc_lim_y(acc_lim_y),
        acc_lim_theta(acc_lim_theta),
        acc_lim_trans(acc_lim_trans),
        xy_goal_tolerance(xy_goal_tolerance),
        yaw_goal_tolerance(yaw_goal_tolerance),
        trans_stopped_vel(trans_stopped_vel),
        theta_stopped_vel(theta_stopped_vel) {}

  ~LocalPlannerLimits() {}

  Eigen::Vector3f getAccLimits() {
    Eigen::Vector3f acc_limits;
    acc_limits[0] = acc_lim_x;
    acc_limits[1] = acc_lim_y;
    acc_limits[2] = acc_lim_theta;
    return acc_limits;
  }

  double max_vel_trans;
  double min_vel_trans;
  double max_vel_x;
  double min_vel_x;
  double max_vel_y;
  double min_vel_y;
  double max_vel_theta;
  double min_vel_theta;
  double acc_lim_x;
  double acc_lim_y;
  double acc_lim_theta;
  double acc_lim_trans;
  double xy_goal_tolerance;
  double yaw_goal_tolerance;
  double trans_stopped_vel;
  double theta_stopped_vel;
};

}

#endif // !AI_ROBOT_LOCAL_PLANNER_LIMITS_H_
