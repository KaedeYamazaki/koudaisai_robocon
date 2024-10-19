#pragma once

#include <cstdint>
#include <string>

namespace config {
static constexpr int serial_baudrate = 115200;
static constexpr std::string_view ps4_controller_mac = "90:38:0C:EB:09:F2";

static constexpr uint8_t pwm_resulution_bit = 8;
static constexpr uint32_t pwm_frequency = 490;

static constexpr uint8_t right_motor_pwm_controller_channel = 0;
static constexpr uint8_t left_motor_pwm_controller_channel = 1;
static constexpr uint8_t arm_motor_pwm_controller_channel = 2;

static constexpr uint8_t right_motor_pwm_pin = 23;
static constexpr uint8_t left_motor_pwm_pin = 22;

static constexpr uint8_t left_motor_in1_pin = 4;
static constexpr uint8_t left_motor_in2_pin = 0;

static constexpr uint8_t right_motor_in1_pin = 19;
static constexpr uint8_t right_motor_in2_pin = 18;

static constexpr bool left_motor_rev = false;
static constexpr bool righr_motor_rev = true;

static constexpr uint8_t doi_led_pin = 27;

namespace servo_motor {
    static constexpr uint16_t target_min = 4 * 500;
    static constexpr uint16_t target_max = 4 * 2500;

    static constexpr uint16_t servo_range_deg = 270;  // 0 to 270 degree

    static constexpr float deg_per_target = static_cast<float>(target_max - target_min) / (servo_range_deg);

    static constexpr uint16_t target_neutral = 4 * 1500;

    namespace left {
        static constexpr uint8_t channel = 0;
        static constexpr uint16_t target_min = 4 * 1100;
        static constexpr uint16_t target_max = target_neutral + 90 * deg_per_target;
        static constexpr int target_offset = 0;
        static constexpr bool rev = false;
    }  // namespace left

    namespace right {
        static constexpr uint8_t channel = 1;
        static constexpr uint16_t target_min = target_neutral - 90 * deg_per_target;
        static constexpr uint16_t target_max = 4 * 1700;
        static constexpr int target_offset = 0;
        static constexpr bool rev = true;
    }  // namespace right

    namespace bucket {
        static constexpr uint8_t channel = 2;
        static constexpr uint16_t target_min = 4 * 1389;
        static constexpr uint16_t target_max = 4 * 2500;
        static constexpr int target_offset = 0;
        static constexpr bool rev = false;
        // namespace right
    }  // namespace bucket
}  // namespace servo_motor

namespace arm {
    static constexpr float left_arm_close_angle = 270.f;
    static constexpr float left_arm_open_angle = 0.f;
    static constexpr float right_arm_close_angle = 270.f;
    static constexpr float right_arm_open_angle = 0.f;
}  // namespace arm

namespace bucket {
    static constexpr float scoop_angle = -10.f;
    static constexpr float lift_angle = 0.f;
    static constexpr float turn_over_angle = 270.f;
}  // namespace bucket
}  // namespace config