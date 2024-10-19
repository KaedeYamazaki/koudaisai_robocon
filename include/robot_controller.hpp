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
    RobotController(const std::string &controller_mac, robot::Robot &robot, const uint8_t doi_led_pin);
    int setup();
    void cycle();

private:
    const std::string controller_mac_;
    robot::Robot &robot_;
    robot::RobotState state_;
    int bucket_value_count_;
    uint8_t doi_led_pin_;
    const float slow_down_{0.3f};

    void semi_auto();
    void manual();
    void drive();
};
}  // namespace robot