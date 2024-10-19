#include <PS4Controller.h>

#include "robot.hpp"

namespace robot {
enum class RobotState {
    setup,
    standby,
    manual,
    semi_auto,
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
    int bucket_value_count_;

    void semi_auto();
    void manual();
    void drive();
};
}  // namespace robot