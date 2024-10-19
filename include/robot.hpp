#pragma once

#include "arm_driver.hpp"
#include "bucket.hpp"
#include "chassis_driver.hpp"

namespace robot {
class Robot {
public:
    Robot() = default;
    ~Robot() = default;
    Robot(chassis_driver::ChassisDriver &chassis, arm::ArmDriver &arm, bucket::Bucket &bucket);

    int setup();

    void drive(float left_duty, float right_duty);

    void arm_degree(float left_degree, float right_degree);

    void arm_close();

    void arm_open();

    void bucket_degree(float degree);

    void bucket_scoop();

    void bucket_lift();

    void bucket_turn_over();

    chassis_driver::ChassisDriver &chassis_;
    arm::ArmDriver &arm_;
    bucket::Bucket &bucket_;
};

}  // namespace robot
