#include "chassis_driver.hpp"

namespace chassis_driver {
ChassisDriver::ChassisDriver(motor_driver::MotorDriver &left_motor_driver,
                             motor_driver::MotorDriver &right_motor_driver)
    : left_motor_driver_(left_motor_driver), right_motor_driver_(right_motor_driver) {}

int ChassisDriver::setup() {
    int i = 0;
    i += left_motor_driver_.setup();
    i += right_motor_driver_.setup();
    return i;
}

void ChassisDriver::drive(float left_duty, float right_duty) {
    if (left_duty == 0.f) {
        left_motor_driver_.brake();
    } else {
        left_motor_driver_.drive(left_duty);
    }
    if (right_duty == 0.f) {
        right_motor_driver_.brake();
    } else {
        right_motor_driver_.drive(right_duty);
    }
}
}  // namespace chassis_driver