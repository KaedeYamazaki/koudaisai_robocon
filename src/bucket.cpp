#include "bucket.hpp"

namespace bucket {
Bucket::Bucket(servo_motor_driver::ServoMotorDriver &bucket_servo, const BucketConfig &config)
    : bucket_servo_(bucket_servo), config_(config) {};

int Bucket::setup() { return bucket_servo_.setup(); }

void Bucket::degree(float degree) { bucket_servo_.degree(degree); }

void Bucket::scoop() { bucket_servo_.degree(config_.scoop_angle); }

void Bucket::lift() { bucket_servo_.degree(config_.lift_angle); }

void Bucket::turn_over() { bucket_servo_.degree(config_.turn_over_angle); }

}  // namespace bucket