#pragma once

#include "servo_motor_driver.hpp"

namespace arm {
struct ArmConfig {
    ArmConfig(const float left_arm_close_angle, const float left_arm_open_angle, const float right_arm_close_angle,
              const float right_arm_open_angle)
        : left_arm_close_angle(left_arm_close_angle),
          left_arm_open_angle(left_arm_open_angle),
          right_arm_close_angle(right_arm_close_angle),
          right_arm_open_angle(right_arm_open_angle) {};

    const float left_arm_close_angle;
    const float left_arm_open_angle;
    const float right_arm_close_angle;
    const float right_arm_open_angle;
};

class ArmDriver {
public:
    ArmDriver(servo_motor_driver::ServoMotorDriver &left_arm_servo,
              servo_motor_driver::ServoMotorDriver &right_arm_servo, const arm::ArmConfig &config);
    ArmDriver() = default;
    ~ArmDriver() = default;

    int setup();

    void degree(float left_degree, float right_degree);

    void close();

    void open();

    ArmConfig config() const { return config_; }

private:
    servo_motor_driver::ServoMotorDriver &left_arm_servo_;
    servo_motor_driver::ServoMotorDriver &right_arm_servo_;
    const arm::ArmConfig config_;
};
}  // namespace arm
