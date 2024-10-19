/* Include */
#include <Arduino.h>
#include <PS4Controller.h>
#include <PololuMaestro.h>
#include <Wire.h>

#include "arm_driver.hpp"
#include "bucket.hpp"
#include "chassis_driver.hpp"
#include "config.hpp"
#include "robot.hpp"
#include "robot_controller.hpp"
#include "servo_motor_driver.hpp"

/* Define */
#include <SoftwareSerial.h>

#define DEBUG false

// motor setup
motor_driver::MotorDriverConfig left_motor_config(config::motor::pwm_resulution_bit, config::motor::pwm_frequency,
                                                  config::motor::left_motor_pwm_controller_channel,
                                                  config::motor::left_motor_pwm_pin, config::motor::left_motor_in1_pin,
                                                  config::motor::left_motor_in2_pin);

motor_driver::MotorDriverConfig right_motor_config(config::motor::pwm_resulution_bit, config::motor::pwm_frequency,
                                                   config::motor::right_motor_pwm_controller_channel,
                                                   config::motor::right_motor_pwm_pin,
                                                   config::motor::right_motor_in1_pin,
                                                   config::motor::right_motor_in2_pin);

// motor_driver::MotorDriverConfig right_motor_config(config::pwm_resulution_bit, config::pwm_frequency,
//                                                    config::right_motor_pwm_controller_channel,
//                                                    config::right_motor_pwm_pin, config::right_motor_in1_pin,
//                                                    config::right_motor_in2_pin);
// motor_driver::MotorDriverConfig left_motor_config(config::pwm_resulution_bit, config::pwm_frequency,
//                                                   config::left_motor_pwm_controller_channel,
//                                                   config::left_motor_pwm_pin, config::left_motor_in1_pin,
//                                                   config::left_motor_in2_pin);

motor_driver::MotorDriver right_motor_driver(right_motor_config);
motor_driver::MotorDriver left_motor_driver(left_motor_config);

chassis_driver::ChassisDriver chassis(left_motor_driver, right_motor_driver);

servo_motor_driver::ServoMotorConfig left_arm_config(
    config::servo_motor::left::channel, config::servo_motor::target_neutral, config::servo_motor::left::target_offset,
    config::servo_motor::left::target_min, config::servo_motor::left::target_max, config::servo_motor::deg_per_target,
    config::servo_motor::left::rev);

servo_motor_driver::ServoMotorConfig right_arm_config(
    config::servo_motor::right::channel, config::servo_motor::target_neutral, config::servo_motor::right::target_offset,
    config::servo_motor::right::target_min, config::servo_motor::right::target_max, config::servo_motor::deg_per_target,
    config::servo_motor::right::rev);

servo_motor_driver::ServoMotorConfig bucket_servo_config(
    config::servo_motor::bucket::channel, config::servo_motor::target_neutral,
    config::servo_motor::bucket::target_offset, config::servo_motor::bucket::target_min,
    config::servo_motor::bucket::target_max, config::servo_motor::deg_per_target, config::servo_motor::bucket::rev);

MicroMaestro maestro(Serial1);

servo_motor_driver::ServoMotorDriver left_arm_servo(maestro, left_arm_config);
servo_motor_driver::ServoMotorDriver right_arm_servo(maestro, right_arm_config);
servo_motor_driver::ServoMotorDriver bucket_servo(maestro, bucket_servo_config);

arm::ArmConfig arm_config(config::arm::left_arm_close_angle, config::arm::left_arm_open_angle,
                          config::arm::right_arm_close_angle, config::arm::right_arm_open_angle);
arm::ArmDriver arm_driver(left_arm_servo, right_arm_servo, arm_config);

bucket::BucketConfig bucket_config(config::bucket::scoop_angle, config::bucket::lift_angle,
                                   config::bucket::turn_over_angle);
bucket::Bucket bucket_arm(bucket_servo, bucket_config);

robot::Robot ibonoito(chassis, arm_driver, bucket_arm);
robot::RobotController ibonoito_controller(static_cast<std::string>(config::serial::ps4_controller_mac), ibonoito,
                                           config::doi_led_pin);

void setup() {
    Serial.begin(config::serial::baudrate);
    Serial.println("start setup");
    Serial.println("setup maestro serial");
    Serial1.begin(config::servo_motor::serial_baudrate, SERIAL_8N1, config::servo_motor::serial_rx,
                  config::servo_motor::serial_tx);

    // setup
    Serial.println("setup robot");
    ibonoito_controller.setup();

    left_motor_driver.rev(config::motor::left_motor_rev);
    right_motor_driver.rev(config::motor::righr_motor_rev);

    delay(100);
}

void loop() {
    ibonoito_controller.cycle();
    delay(50);
}
