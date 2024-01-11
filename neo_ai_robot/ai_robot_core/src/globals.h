#ifndef AI_ROBOT_GLOBALS_H
#define AI_ROBOT_GLOBALS_H

#include <mutex>

namespace ai_robot {

namespace g {

extern std::mutex oa_lock;
extern std::mutex mtx;
extern bool in_man_mode;
extern bool man_test_mode;

}
}

#endif // !AI_ROBOT_GLOBALS_H
