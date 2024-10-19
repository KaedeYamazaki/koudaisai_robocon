/* Include */
#include <Arduino.h>
#include <PS4Controller.h>
#include <PololuMaestro.h>
#include <Wire.h>

#include "arm_driver.hpp"
#include "chassis_driver.hpp"
#include "config.hpp"
#include "servo_motor_driver.hpp"

/* Define */
#include <SoftwareSerial.h>

#define DEBUG true

// SoftwareSerial maestroSerial(0, 1);
static const int indicator = 27;

float left_duty = 0;
float right_duty = 0;

// motor setup
motor_driver::MotorDriverConfig right_motor_config(config::pwm_resulution_bit, config::pwm_frequency,
                                                   config::right_motor_pwm_controller_channel,
                                                   config::right_motor_pwm_pin, config::right_motor_in1_pin,
                                                   config::right_motor_in2_pin);
motor_driver::MotorDriverConfig left_motor_config(config::pwm_resulution_bit, config::pwm_frequency,
                                                  config::left_motor_pwm_controller_channel, config::left_motor_pwm_pin,
                                                  config::left_motor_in1_pin, config::left_motor_in2_pin);

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

servo_motor_driver::ServoMotorConfig bucket_config(
    config::servo_motor::bucket::channel, config::servo_motor::target_neutral,
    config::servo_motor::bucket::target_offset, config::servo_motor::bucket::target_min,
    config::servo_motor::bucket::target_max, config::servo_motor::deg_per_target, config::servo_motor::bucket::rev);

MicroMaestro maestro(Serial1);
servo_motor_driver::ServoMotorDriver left_arm_servo(maestro, left_arm_config);
servo_motor_driver::ServoMotorDriver right_arm_servo(maestro, right_arm_config);
servo_motor_driver::ServoMotorDriver bucket_servo(maestro, bucket_config);

arm::ArmConfig arm_config(0.f, 0.f, 0.f, 0.f);
arm::ArmDriver arm_driver(left_arm_servo, right_arm_servo, arm_config);

void setup() {
    Serial.begin(config::serial_baudrate);
    Serial.println("start setup");
    Serial.println("setup maestro serial");
    Serial1.begin(9600, SERIAL_8N1, 22, 23);

    // setup
    // chassis.setup();
    // left_motor_driver.rev(config::left_motor_rev);
    // right_motor_driver.rev(config::righr_motor_rev);

    arm_driver.setup();

    pinMode(indicator, OUTPUT);

    // PS4 controller setup
    PS4.begin(config::ps4_controller_mac.data());
    delay(100);
}

void loop() {
    if (PS4.isConnected()) {
        digitalWrite(indicator, HIGH);

        float R = PS4.RStickY();
        float L = PS4.LStickY();

        if (abs(R) <= 15) R = 0;
        if (abs(L) <= 15) L = 0;

        float left_angle = static_cast<float>(config::servo_motor::servo_range_deg * L / INT8_MAX);
        float right_angle = static_cast<float>(config::servo_motor::servo_range_deg * R / INT8_MAX);

        arm_driver.degree(left_angle, right_angle);

        Serial.print(" left_angle: ");
        Serial.print(left_angle);
        Serial.print(" right_angle: ");
        Serial.println(right_angle);

    } else {
        Serial.println("PS4 controller is not connected");
        digitalWrite(indicator, LOW);
    }
    delay(10);
}
