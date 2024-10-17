#include "arm_driver.hpp"

namespace arm {
ArmDriver::ArmDriver(servo_motor_driver::ServoMotorDriver &left_arm_servo,
                     servo_motor_driver::ServoMotorDriver &right_arm_servo)
    : left_arm_servo_(left_arm_servo), right_arm_servo_(right_arm_servo) {}
}  // namespace arm