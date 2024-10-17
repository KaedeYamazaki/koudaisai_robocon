#pragma once

#include "servo_motor_driver.hpp"

namespace arm {
class ArmDriver {
public:
    ArmDriver(servo_motor_driver::ServoMotorDriver &left_arm_servo,
              servo_motor_driver::ServoMotorDriver &right_arm_servo);
    ArmDriver() = default;
    ~ArmDriver() = default;

    int setup();

    void degree(float left_degree, float right_degree);

private:
    servo_motor_driver::ServoMotorDriver &left_arm_servo_;
    servo_motor_driver::ServoMotorDriver &right_arm_servo_;
};
}  // namespace armd