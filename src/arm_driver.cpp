#include "arm_driver.hpp"

namespace arm {
ArmDriver::ArmDriver(servo_motor_driver::ServoMotorDriver &left_arm_servo,
                     servo_motor_driver::ServoMotorDriver &right_arm_servo, const arm::ArmConfig &config)
    : left_arm_servo_(left_arm_servo), right_arm_servo_(right_arm_servo), config_(config) {};

int ArmDriver::setup() {
    int i = 0;
    i += left_arm_servo_.setup();
    i += right_arm_servo_.setup();
    return i;
}

void ArmDriver::degree(float left_degree, float right_degree) {
    left_arm_servo_.degree(left_degree);
    right_arm_servo_.degree(right_degree);
}

void ArmDriver::close() {
    left_arm_servo_.degree(config_.left_arm_close_angle);
    right_arm_servo_.degree(config_.right_arm_close_angle);
}

void ArmDriver::open() {
    left_arm_servo_.degree(config_.left_arm_open_angle);
    right_arm_servo_.degree(config_.right_arm_open_angle);
}

}  // namespace arm