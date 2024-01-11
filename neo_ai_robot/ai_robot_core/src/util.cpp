#include "util.h"

#include <cmath>

namespace ai_robot {
namespace util {

// Calculates distance from origin to the given point.
double distance2d(double x1, double y1, double x2, double y2) {
  return std::hypot(x2 - x1, y2 - y1);
}

double distance2d(const double p1[2], const double p2[2]) {
  return distance2d(p1[0], p1[1], p2[0], p2[1]);
}

// Gets distance from a 3-dimensional point
double distance3d(double x1, double y1, double z1, double x2, double y2, double z2) {
  return std::sqrt(std::pow((x2 - x1), 2) + std::pow((y2 - y1), 2) + std::pow((z2 - z1), 2));
}

double distance3d(const double p1[3], const double p2[3]) {
  return distance3d(p1[0], p1[1], p1[2], p2[0], p2[1], p2[2]);
}

// Calculates manhattan distance between two 2-d points.
double mdistance2d(double x1, double y1, double x2, double y2) {
  return fabs(x2 - x1) + fabs(y2 - y1);
}

double mdistance2d(const double p1[], const double p2[]) {
  return fabs(p2[0] - p1[0]) + fabs(p2[1] - p1[1]);
}

// Converts quaternion to Euler angles
void quat2eul(double& roll, double& pitch, double& yaw, double w, double x, double y, double z) {
  double tmp = 2 * (w * y - z * x);

  if (fabs(w) + fabs(x) + fabs(y) + fabs(z) < 0.1) {
    roll = pitch = yaw = 0;
  }

  roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  if (tmp > 1)
    pitch = asin(1.0);
  else if (tmp < -1.0)
    pitch = asin(-1.0);
  else
    pitch = asin(tmp);
  yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

void quat2eul(double& roll, double& pitch, double& yaw, const double quat[]) {
  quat2eul(roll, pitch, yaw, quat[0], quat[1], quat[2], quat[3]);
}

void quat2eul(double rpy[], const double quat[]) {
  quat2eul(rpy[0], rpy[1], rpy[2], quat);
}

// Conerts Euler angles to quaternion
void eul2quat(double quat[], const double roll, const double pitch, const double yaw) {
  double cos_roll = std::cos(roll * .5);
  double sin_roll = std::sin(roll * .5);
  double cos_pitch = std::cos(pitch * .5);
  double sin_pitch = std::sin(pitch * .5);
  double cos_yaw = std::cos(yaw * .5);
  double sin_yaw = std::sin(yaw * .5);

  quat[0] = cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw; // w
  quat[1] = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw; // x
  quat[2] = cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw; // y
  quat[3] = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw; // z
}

// Converts quaternion to rotation matrix
void quat2rotm(double rotm[], const double quat[]) {
  double w = quat[0], x = quat[1], y = quat[2], z = quat[3];
  rotm[0] = w * w + x * x - y * y - z * z;
  rotm[1] = 2 * (x * y - w * z);
  rotm[2] = 2 * (x * z + w * y);
  rotm[3] = 2 * (x * y + w * z);
  rotm[4] = w * w - x * x + y * y - z * z;
  rotm[5] = 2 * (y * z - w * x);
  rotm[6] = 2 * (x * z - w * y);
  rotm[7] = 2 * (y * z + w * x);
  rotm[8] = w * w - x * x - y * y + z * z;
}

// Converts rotation matrix to quaternion
void rotm2quat(double quat[], const double rotm[]) {
  double s;
  double tr = rotm[0] + rotm[4] + rotm[8];

  if (tr <= 0) { // fabs(quat[0]) < 0.001
    if (rotm[0] > rotm[4] && rotm[0] > rotm[8]) {
      s = sqrt(1 + rotm[0] - rotm[4] - rotm[8]) * 2;
      quat[0] = (rotm[7] - rotm[5]) / s;
      quat[1] = s / 4;
      quat[2] = (rotm[1] + rotm[3]) / s;
      quat[3] = (rotm[2] + rotm[6]) / s;
    } else if (rotm[4] > rotm[8]) {
      s = sqrt(1 - rotm[0] + rotm[4] - rotm[8]) * 2;
      quat[0] = (rotm[2] - rotm[6]) / s;
      quat[1] = (rotm[1] + rotm[3]) / s;
      quat[2] = s / 4;
      quat[3] = (rotm[5] + rotm[7]) / s;
    } else {
      s = sqrt(1 - rotm[0] - rotm[4] + rotm[8]) * 2;
      quat[0] = (rotm[3] - rotm[1]) / s;
      quat[1] = (rotm[2] + rotm[6]) / s;
      quat[2] = (rotm[5] + rotm[7]) / s;
      quat[3] = s / 4;
    }
    return;
  }

  quat[0] = sqrt(1 + rotm[0] + rotm[4] + rotm[8]) / 2;
  quat[1] = (rotm[7] - rotm[5]) / 4 / quat[0];
  quat[2] = (rotm[2] - rotm[6]) / 4 / quat[0];
  quat[3] = (rotm[3] - rotm[1]) / 4 / quat[0];
}

void body2nwu(double& nwux, double& nwuy, double& nwuz, double bodyx, double bodyy, double bodyz, const double r[], const double t[]) {
  nwux = r[0] * bodyx + r[1] * bodyy + r[2] * bodyz + t[0];
  nwuy = r[3] * bodyx + r[4] * bodyy + r[5] * bodyz + t[1];
  nwuz = r[6] * bodyx + r[7] * bodyy + r[8] * bodyz + t[2];
}

void body2nwu(double nwu_point[], const double body_point[], const double r[], const double t[]) {
  body2nwu(nwu_point[0], nwu_point[1], nwu_point[2], body_point[0], body_point[1], body_point[2], r, t);
}

void nwu2body(double& bodyx, double& bodyy, double& bodyz, double nwux, double nwuy, double nwuz, const double r[], const double t[]) {
  double dx = nwux - t[0];
  double dy = nwuy - t[1];
  double dz = nwuz - t[2];
  bodyx = r[0] * dx + r[3] * dy + r[6] * dz;
  bodyy = r[1] * dx + r[4] * dy + r[7] * dz;
  bodyz = r[2] * dx + r[5] * dy + r[8] * dz;
}

void nwu2body(double body_point[], const double nwu_point[], const double r[], const double t[]) {
  body2nwu(body_point[0], body_point[1], body_point[2], nwu_point[0], nwu_point[1], nwu_point[2], r, t);
}

// Dot product of two rotation matrices
void dotRotM(double prod[], const double m1[], const double m2[]) {
  for (size_t i = 0; i < 3; ++i)
    for (size_t j = 0; j < 3; ++j) 
      for (size_t k = 0; k < 3; ++k)
        prod[i * 3 + j] += m1[i * 3 + k] * m2[k * 3 + j];
}

// Dot product of the first matrix transpose and the second matrix
void dotRotM1T(double prod[], const double m1[], const double m2[]) {
  prod[0] = m1[0] * m2[0] + m1[3] * m2[3] + m1[6] * m2[6];
  prod[1] = m1[0] * m2[1] + m1[3] * m2[4] + m1[6] * m2[7];
  prod[2] = m1[0] * m2[2] + m1[3] * m2[5] + m1[6] * m2[8];
  prod[3] = m1[1] * m2[0] + m1[4] * m2[3] + m1[7] * m2[6];
  prod[4] = m1[1] * m2[1] + m1[4] * m2[4] + m1[7] * m2[7];
  prod[5] = m1[1] * m2[2] + m1[4] * m2[5] + m1[7] * m2[8];
  prod[6] = m1[2] * m2[0] + m1[5] * m2[3] + m1[8] * m2[6];
  prod[7] = m1[2] * m2[1] + m1[5] * m2[4] + m1[8] * m2[7];
  prod[8] = m1[2] * m2[2] + m1[5] * m2[5] + m1[8] * m2[8];
}

// Cross product of two quaternions
void crossQuat(double cprod[], const double quat1[], const double quat2[]) {
  cprod[0] = quat1[0] * quat2[0] - quat1[1] * quat2[1] - quat1[2] * quat2[2] - quat1[3] * quat2[3];
  cprod[1] = quat1[0] * quat2[1] - quat1[1] * quat2[0] - quat1[2] * quat2[3] - quat1[3] * quat2[2];
  cprod[2] = quat1[0] * quat2[2] - quat1[2] * quat2[0] - quat1[3] * quat2[1] - quat1[1] * quat2[3];
  cprod[3] = quat1[0] * quat2[3] - quat1[3] * quat2[0] - quat1[1] * quat2[2] - quat1[2] * quat2[1];
}


namespace {
constexpr double kEpsilon = 1e-6;
}

bool isEqualFloat(double x, double y) {
  return std::fabs(x - y) < kEpsilon ? true : false;
}


bool enuHdg(double& hdg, double p1_x, double p1_y, double p2_x, double p2_y) {
  if (isEqualFloat(p1_x, p2_x)) { // has same x value
    if (isEqualFloat(p1_y, p2_y)) /* two points coincide */
        return false;
    else
      hdg = (p2_y > p1_y) ? 0 : M_PI;    
  } else {
    if (isEqualFloat(p2_y, p1_y)) {
        hdg = (p2_x > p1_x) ? M_PI_2 : -M_PI_2;
    } else {
      double dx = p2_x - p1_x;
      double dy = p2_y - p1_y;
      double hyp = std::hypot(dx, dy);
      double rad = std::acos(dx / hyp);
      if (rad < M_PI_2)
          hdg = dy > 0 ? rad - M_PI_2 : -(M_PI_2 + rad);
      else
          hdg = dy > 0 ? rad - M_PI_2 : 3 * M_PI_2 - rad;
    }
  }
  return true;
}

double normAngle(double z) {
  return atan2(sin(z), cos(z));
}

double getAngleDiff(double a, double b) {
  double d1, d2;
  // make a and b in range of (-180, 180]
  a = normAngle(a);
  b = normAngle(b);
  // d1 is in range of (-360, 360), need to make it in (-180, 180]
  d1 = a - b;
  d2 = 2 * M_PI - fabs(d1);
  if (d1 > 0)
    d2 *= -1.0;
  if (fabs(d1) < fabs(d2))
    return d1;
  else
    return d2;
}

}
}
