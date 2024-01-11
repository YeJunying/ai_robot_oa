#include <iostream>
#include <iomanip>

#include "../bluetooth.h"

int main() {
  double m1[2] = {13519942.879317075, 3614351.288322719};
  double m2[2] = {13519969.46712373, 3614351.2136771837};
  double m3[2] = {13519943.030940821, 3614374.5497376984};
  double p1[2] = {40.0, 20.0};
  double p2[2] = {887.0, 20.0};
  double p3[2] = {40.0, 762.0};
  double p4[2] = {18.503357 / 0.03464, 14.004132 / 0.03464};
  // double p5[2] = {15.358902, 16.029341};
  double p5[2] = {10.358902 / 0.03464, 22.029341 / 0.03464};

  ai_robot::bluetooth::Transform::merca2plane(m1[0], m1[1]);
  std::cout << "m1 -> p1: " << m1[0] << ", " << m1[1] << '\n';
  ai_robot::bluetooth::Transform::merca2plane(m2[0], m2[1]);
  std::cout << "m2 -> p1: " << m2[0] << ", " << m2[1] << '\n';
  ai_robot::bluetooth::Transform::merca2plane(m3[0], m3[1]);
  std::cout << "m3 -> p1: " << m3[0] << ", " << m3[1] << '\n';

  ai_robot::bluetooth::Transform::plane2merca(p1[0], p1[1]);
  std::cout << "p1 -> m1: " << std::fixed << std::setprecision(10) << p1[0] << ", " << p1[1] << '\n';
  ai_robot::bluetooth::Transform::plane2merca(p2[0], p2[1]);
  std::cout << "p2 -> m2: " << std::fixed << std::setprecision(10) << p2[0] << ", " << p2[1] << '\n';
  ai_robot::bluetooth::Transform::plane2merca(p3[0], p3[1]);
  std::cout << "p3 -> m3: " << std::fixed << std::setprecision(10) << p3[0] << ", " << p3[1] << '\n';
  ai_robot::bluetooth::Transform::plane2merca(p4[0], p4[1]);
  std::cout << "p4 -> m4: " << std::fixed << std::setprecision(10) << p4[0] << ", " << p4[1] << '\n';
  ai_robot::bluetooth::Transform::plane2merca(p5[0], p5[1]);
  std::cout << "p5 -> m5: " << std::fixed << std::setprecision(10) << p5[0] << ", " << p5[1] << '\n';

  return 0;
}
