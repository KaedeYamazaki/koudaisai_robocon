/* Include */
#include <Arduino.h>
#include <PS4Controller.h>

#include "chassis_driver.hpp"
#include "config.hpp"

/* Define */
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

void setup() {
    Serial.begin(config::serial_baudrate);
    Serial.println("start setup");

    // setup
    chassis.setup();
    left_motor_driver.rev(config::left_motor_rev);
    right_motor_driver.rev(config::righr_motor_rev);

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

        left_duty = static_cast<float>(L / INT8_MAX);
        right_duty = static_cast<float>(R / INT8_MAX);
        left_motor_driver.drive(left_duty);
        right_motor_driver.drive(right_duty);

#ifdef DEBUG
        Serial.print("L: ");
        Serial.print(L);
        Serial.print(" R: ");
        Serial.print(R);
        Serial.println(" ");

        Serial.print(" left_duty: ");
        Serial.print(left_duty);
        Serial.print(" right_duty: ");
        Serial.println(right_duty);
#endif
    } else {
        Serial.println("PS4 controller is not connected");
        digitalWrite(indicator, LOW);
    }
    delay(10);
}
