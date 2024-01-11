#ifndef AI_ROBOT_BLUETOOTH_H
#define AI_ROBOT_BLUETOOTH_H

#include <Eigen/Dense>

namespace ai_robot {

namespace bluetooth {

struct Waypoint {
  Waypoint(double x, double y, int level) : x(x), y(y), level(level) {}
  double x;
  double y;
  int level; // which floor
};

class Transform {
public:
  Transform() = default;
  virtual ~Transform() = default;

public:
  static void merca2plane(double in_x, double in_y, double& out_x, double& out_y, double scale=1.0);
  static void merca2plane(double& x, double& y, double scale=1.0);
  static void plane2merca(double in_x, double in_y, double& out_x, double& out_y, double scale=1.0);
  static void plane2merca(double& x, double& y, double scale=1.0);
  static void printRotationMatrix();

private:
  static Eigen::Matrix2d rotm_; // rotation matrix from mercator to map
  static Eigen::Matrix2d rotm_inv_; // inversion of rotation matrix from mercator to map
  static Eigen::Vector2d tr_; // translation matrix from mercator to map
};

}
}

#endif // !AI_ROBOT_BLUEWAYPOINT_H
