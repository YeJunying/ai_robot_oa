#ifndef AI_ROBOT_UTIL
#define AI_ROBOT_UTIL

#include <cmath>
#include <string>

// #include <opencv2/opencv.hpp>

namespace ai_robot {
namespace util {

// std::string stripLeadingSlash(const std::string& in);
double normAngle(double z);
double getAngleDiff(double a, double b);

// std::string stripLeadingSlash(const std::string&);
double distance2d(double x1, double y1, double x2, double y2);
double distance2d(const double p1[2], const double p2[2]);
double distance3d(double x1, double y1, double z1, double x2, double y2, double z2);
double distance3d(const double p1[3], const double p2[3]);
void quat2eul(double& roll, double& pitch, double& yaw, double w, double x, double y, double z);
void quat2eul(double& roll, double& pitch, double& yaw, const double quat[]);
void quat2eul(double rpy[], const double quat[]);
void eul2quat(double quat[], const double roll, const double pitch, const double yaw);
void quat2rotm(double rotm[], const double quat[]);
void rotm2quat(double quat[], const double rotm[]);
void body2nwu(double& nwux, double& nwuy, double& nwuz, double bodyx, double bodyy, double bodyz, const double r[], const double t[]);
void body2nwu(double nwu_point[], const double body_point[], const double r[], const double t[]);
void nwu2body(double& bodyx, double& bodyy, double& bodyz, double nwux, double nwuy, double nwuz, const double r[], const double t[]);
void nwu2body(double body_point[], const double nwu_point[], const double r[], const double t[]);
void dotRotM(double prod[], const double m1[], const double m2[]);
void dotRotM1T(double prod[], const double m1[], const double m2[]);
void crossQuat(double prod[], const double quat1[], const double quat2[]);
bool isEqualFloat(double x, double y);
bool enuHdg(double& hdg, double p1_x, double p1_y, double p2_x, double p2_y);

// Converts OpenCV Mat to quaternion
// template <typename T>
// void _cvmat2quat(double quat[], const cv::Mat mat) {
//   quat[0] = sqrt(1 + mat.at<T>(0, 0) + mat.at<T>(1, 1) + mat.at<T>(2, 2)) / 2;
//   quat[1] = (mat.at<T>(2, 1) - mat.at<T>(1, 2)) / (4 + quat[0]);
//   quat[2] = (mat.at<T>(0, 2) - mat.at<T>(2, 0)) / (4 + quat[0]);
//   quat[3] = (mat.at<T>(1, 0) - mat.at<T>(0, 1)) / (4 + quat[0]);
// }

// template <int T>
// void cvmat2quat(double quat[], const cv::Mat mat) {
//   if (T == CV_32F)
//     _cvmat2quat<float>(quat, mat);
//   else if (T == CV_64F)
//     _cvmat2quat<double>(quat, mat);
// }

inline double rad2deg(double rad) {
  return rad * 180.0 / M_PI;
}

inline double deg2rad(double deg) {
  return deg * M_PI / 180;
}

}
}

#endif // !AI_ROBOT_UTIL
