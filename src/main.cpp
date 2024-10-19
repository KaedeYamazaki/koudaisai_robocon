/* Include */
#include <Arduino.h>
#include <PS4Controller.h>
#include <PololuMaestro.h>
#include <Wire.h>

#include "chassis_driver.hpp"
#include "config.hpp"
#include "servo_motor_driver.hpp"
#include "arm_driver.hpp"


/* Define */
#include <SoftwareSerial.h>
SoftwareSerial maestroSerial(0, 1);
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
    config::servo_motor::left::channel, config::servo_motor::target_neutral,
    config::servo_motor::left::target_offset, config::servo_motor::left::target_min,
    config::servo_motor::left::target_max, config::servo_motor::deg_per_target, config::servo_motor::left::rev);

servo_motor_driver::ServoMotorConfig right_arm_config(
    config::servo_motor::right::channel, config::servo_motor::target_neutral,
    config::servo_motor::right::target_offset, config::servo_motor::right::target_min,
    config::servo_motor::right::target_max, config::servo_motor::deg_per_target, config::servo_motor::right::rev);

MicroMaestro maestro(maestroSerial);
servo_motor_driver::ServoMotorDriver left_arm_servo(maestro, left_arm_config);
servo_motor_driver::ServoMotorDriver right_arm_servo(maestro, right_arm_config);

void setup() {
    Serial.begin(config::serial_baudrate);
    Serial.println("start setup");
    Serial.println("setup maestro serial");
    // maestroSerial.begin(9600);

    // setup
    chassis.setup();
    left_motor_driver.rev(config::left_motor_rev);
    right_motor_driver.rev(config::righr_motor_rev);

    // left_arm_servo.setup();
    // right_arm_servo.setup();

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

        // left_arm_servo.degree(left_angle);
        // right_arm_servo.degree(right_angle);

        // left_duty = static_cast<float>(L / INT8_MAX);
        // right_duty = static_cast<float>(R / INT8_MAX);
        // left_motor_driver.drive(left_duty);
        // right_motor_driver.drive(right_duty);

#ifdef DEBUG
        Serial.print("L: ");
        Serial.print(L);
        Serial.print(" R: ");
        Serial.print(R);
        Serial.println(" ");

        // Serial.print(" left_duty: ");
        // Serial.print(left_duty);
        // Serial.print(" right_duty: ");
        // Serial.println(right_duty);
        Serial.print(" left_angle: ");
        Serial.print(left_angle);
        Serial.print(" right_angle: ");
        Serial.println(right_angle);
#endif
    } else {
        Serial.println("PS4 controller is not connected");
        digitalWrite(indicator, LOW);
    }
    delay(10);
}
