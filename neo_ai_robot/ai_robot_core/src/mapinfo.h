#ifndef AI_ROBOT_MAPINFO_H
#define AI_ROBOT_MAPINFO_H

struct MapInfo {
  float resolution;
  struct {
    double x;
    double y;
  } origin;
};

#endif // !AI_ROBOT_MAPINFO_H
