#pragma once

#include <cstdint>

namespace motor_driver {

struct MotorDriverConfig {
    MotorDriverConfig(const uint8_t pwm_resulution_bit, const uint32_t pwm_frequency,
                      const uint8_t pwm_controller_channel, const uint8_t pwm_pin, const uint8_t in1_pin,
                      const uint8_t in2_pin)
        : pwm_resulution_bit(pwm_resulution_bit),
          pwm_frequency(pwm_frequency),
          pwm_controller_channel(pwm_controller_channel),
          pwm_pin(pwm_pin),
          in1_pin(in1_pin),
          in2_pin(in2_pin) {}
    const uint8_t pwm_resulution_bit;
    const uint32_t pwm_frequency;
    const uint8_t pwm_controller_channel;
    const uint8_t pwm_pin;
    const uint8_t in1_pin;
    const uint8_t in2_pin;
};

class MotorDriver {
public:
    MotorDriver(const MotorDriverConfig& config);

    ~MotorDriver() = default;

    int setup();

    void rev(bool rev);

    void drive(float duty);

    void brake();

    MotorDriverConfig config() const { return config_; }

private:
    const MotorDriverConfig config_;
    bool is_reversed_{false};
    float limit_duty(float duty) {
        if (duty > 1.f) {
            return 1.f;
        } else if (duty < -1.f) {
            return -1.f;
        }
        return duty;
    };
};

}  // namespace motor_driver
