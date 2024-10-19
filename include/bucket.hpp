#pragma once

#include "servo_motor_driver.hpp"

namespace bucket {
struct BucketConfig {
    BucketConfig(const float scoop_angle, const float lift_angle, const float turn_over_angle)
        : scoop_angle(scoop_angle), lift_angle(lift_angle), turn_over_angle(turn_over_angle) {};

    const float scoop_angle;
    const float lift_angle;
    const float turn_over_angle;
};

class Bucket {
public:
    Bucket() = default;
    ~Bucket() = default;
    Bucket(servo_motor_driver::ServoMotorDriver &bucket_servo, const BucketConfig &config);
    int setup();
    void degree(float degree);
    void scoop();
    void lift();
    void turn_over();
    

private:
    servo_motor_driver::ServoMotorDriver &bucket_servo_;
    const BucketConfig config_;
};
}  // namespace bucket
