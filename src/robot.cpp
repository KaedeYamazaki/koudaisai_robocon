#include "robot.hpp"

namespace robot {
Robot::Robot(chassis_driver::ChassisDriver &chassis, arm::ArmDriver &arm, bucket::Bucket &bucket)
    : chassis_(chassis), arm_(arm), bucket_(bucket) {}

int Robot::setup() {
    int i = 0;
    i += chassis_.setup();
    i += arm_.setup();
    i += bucket_.setup();
    return i;
}

void Robot::drive(float left_duty, float right_duty) { chassis_.drive(left_duty, right_duty); }

void Robot::arm_degree(float left_degree, float right_degree) { arm_.degree(left_degree, right_degree); }

void Robot::arm_close() { arm_.close(); }

void Robot::arm_open() { arm_.open(); }

void Robot::bucket_degree(float degree) { bucket_.degree(degree); }

void Robot::bucket_scoop() { bucket_.scoop(); }

void Robot::bucket_lift() { bucket_.lift(); }

void Robot::bucket_turn_over() { bucket_.turn_over(); }
}  // namespace robot