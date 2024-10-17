#include "motor_driver.hpp"

#include <Arduino.h>
#include <iostream>
#include <sstream>

namespace motor_driver {

MotorDriver::MotorDriver(const MotorDriverConfig& config) : config_(config) {}

int MotorDriver::setup() {
    Serial.println("motor setup");
    Serial.print("  pwm_pin: ");
    Serial.println(config_.pwm_pin);
    Serial.print("  in1_pin: ");
    Serial.println(config_.in1_pin);
    Serial.print("  in2_pin: ");
    Serial.println(config_.in2_pin);
    Serial.print("  pwm_controller_channel: ");
    Serial.println(config_.pwm_controller_channel);
    Serial.print("  pwm_frequency: ");
    Serial.println(config_.pwm_frequency);
    Serial.print("  pwm_resulution_bit: ");
    Serial.println(config_.pwm_resulution_bit);

    pinMode(config_.pwm_pin, OUTPUT);
    pinMode(config_.in1_pin, OUTPUT);
    pinMode(config_.in2_pin, OUTPUT);

    ledcSetup(config_.pwm_controller_channel, config_.pwm_frequency, config_.pwm_resulution_bit);
    ledcAttachPin(config_.pwm_pin, config_.pwm_controller_channel);

    return 1;
};

void MotorDriver::rev(bool rev) { is_reversed_ = rev; }

void MotorDriver::drive(float duty) {
    std::ostringstream oss;

    duty = limit_duty(duty);
    if (duty == 0.f) {
        brake();
        return;
    } else if (duty > 0.f) {
        const auto pin1 = is_reversed_ ? HIGH : LOW;
        const auto pin2 = is_reversed_ ? LOW : HIGH;

        #ifdef DEBUG
        Serial.print("  forward: ");
        Serial.print(" pin1: ");
        Serial.print(pin1);
        Serial.print("  pin2: ");
        Serial.println(pin2);
        #endif

        digitalWrite(config_.in1_pin, pin1);
        digitalWrite(config_.in2_pin, pin2);

    } else {
        const auto pin1 = is_reversed_ ? LOW : HIGH;
        const auto pin2 = is_reversed_ ? HIGH : LOW;

        #ifdef DEBUG
        Serial.print("  backward: ");
        Serial.print(" pin1: ");
        Serial.print(pin1);
        Serial.print("  pin2: ");
        Serial.println(pin2);
        #endif

        digitalWrite(config_.in1_pin, pin1);
        digitalWrite(config_.in2_pin, pin2);
    }
    const uint max_pwm = (2 << (config_.pwm_resulution_bit - 1)) -1;
    const uint input_pwm = abs(static_cast<int>(duty * max_pwm));

    #ifdef DEBUG
    Serial.print("  input_pwm: ");
    Serial.println(input_pwm);
    #endif
    ledcWrite(config_.pwm_controller_channel, input_pwm);
}

void MotorDriver::brake() {
    #ifdef DEBUG
    Serial.println("motor brake: ");
    Serial.print("  in1_pin: ");
    Serial.println(LOW);
    Serial.print("  in2_pin: ");
    Serial.println(LOW);
    Serial.print("  pwm_controller_channel: ");
    Serial.println(config_.pwm_controller_channel);
    Serial.print("  pwm: ");
    Serial.println(LOW);
    #endif

    digitalWrite(config_.in1_pin, LOW);
    digitalWrite(config_.in2_pin, LOW);
    ledcWrite(config_.pwm_controller_channel, LOW);
}
}  // namespace motor_driver