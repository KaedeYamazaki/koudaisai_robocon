#include <cstdint>
#include <string>

namespace config {
    static constexpr uint8_t pwm_resulution_bit = 8;
    static constexpr uint8_t pwm_frequency = 1000;

    static constexpr uint8_t left_motor_pwm_controller_channel = 0;
    static constexpr uint8_t right_motor_pwm_controller_channel = 1;
    static constexpr uint8_t arm_motor_pwm_controller_channel = 2;

    static constexpr uint8_t right_motor_pwm_pin = 23;
    static constexpr uint8_t left_motor_pwm_pin = 22;

    static constexpr uint8_t right_motor_in1_pin = 4;
    static constexpr uint8_t right_motor_in2_pin = 0;

    static constexpr uint8_t left_motor_in1_pin = 19;
    static constexpr uint8_t left_motor_in2_pin = 18;

    static constexpr uint8_t doi_led_pin = 27;

    static constexpr std::string_view ps4_controller_mac = "90:38:0C:EB:09:F2";
}
