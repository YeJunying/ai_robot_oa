#include "bluetooth.h"

#include <cmath>
#include <iomanip>
#include <iostream>

namespace ai_robot {

namespace bluetooth {

Eigen::Matrix2d Transform::rotm_ = (Eigen::Matrix2d() << 31.911474, -0.136371, 0.136371, 31.911474).finished();
// Eigen::Matrix2d Transform:: rotm_ = (Eigen::Matrix2d() << 0.0313361, 0.000133912, -0.000133912, 0.0313361).finished();
Eigen::Matrix2d Transform::rotm_inv_ = Transform::rotm_.inverse();

Eigen::Vector2d Transform::tr_(-430948373.82332796, -117182985.81742199);
// Eigen::Vector2d Transform::tr_(-13519944.81478687, -3614350.66970221);

void Transform::printRotationMatrix() {
  std::cout << rotm_inv_ << '\n';
}

void Transform::merca2plane(double& x, double& y, double scale) {
  merca2plane(x, y, x, y, scale);
}

void Transform::merca2plane(double in_x, double in_y, double& out_x, double& out_y, double scale) {
  Eigen::Vector2d in(in_x, in_y);
  Eigen::Vector2d out = (rotm_ * in + tr_) * scale;
  out_x = out(0);
  out_y = out(1);
}

void Transform::plane2merca(double& x, double& y, double scale) {
  plane2merca(x, y, x, y, scale);
}

void Transform::plane2merca(double in_x, double in_y, double& out_x, double& out_y, double scale) {
  // Eigen::Vector2d tgt = plane2merca(src_x, src_y, scaled);
  // tgt_x = tgt(0);
  // tgt_y = tgt(1);
  Eigen::Vector2d in(in_x, in_y);
  Eigen::Vector2d out = rotm_inv_ * (in / scale - tr_);
  out_x = out(0);
  out_y = out(1);
}

}
}
