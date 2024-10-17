#pragma once

#include "motor_driver.hpp"

namespace chassis_driver {
class ChassisDriver {
public:
    ChassisDriver(motor_driver::MotorDriver &left_motor_driver, motor_driver::MotorDriver &right_motor_driver);
    ChassisDriver() = default;
    ~ChassisDriver() = default;

    int setup();

    void drive(float left_duty, float right_duty);

private:
    motor_driver::MotorDriver &left_motor_driver_;
    motor_driver::MotorDriver &right_motor_driver_;
};
}  // namespace chassis_driver