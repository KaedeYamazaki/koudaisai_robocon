#include <PS4Controller.h>

#include "robot.hpp"

namespace robot {
enum class RobotState {
    setup,
    standby,
    manual,
    semi_auto,
};

struct SemiAuto {
    bool left_arm_close = false;
    bool right_arm_close = false;
};

class RobotController {
public:
    RobotController(const std::string &controller_mac, robot::Robot &robot);
    int setup();
    void cycle();

private:
    const std::string controller_mac_;
    robot::Robot &robot_;
    robot::RobotState state_;
    SemiAuto semi_auto_;
    int bucket_value_count_;

    void semi_auto();
    void manual();

    void drive();
};
}  // namespace robot