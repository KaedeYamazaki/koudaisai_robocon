#pragma once

#include <PololuMaestro.h>

namespace servo_motor_driver {
struct ServoMotorConfig {
    ServoMotorConfig(const uint8_t servo_controller_channel, const uint16_t target_min, const uint16_t target_max,
                     const uint16_t target_neutral, const uint16_t target_offset, const float servo_range_deg)
        : channel(servo_controller_channel),
          target_min(target_min),
          target_max(target_max),
          target_neutral(target_neutral),
          target_offset(target_offset),
          servo_range_deg(servo_range_deg) {}
    const uint8_t channel;
    const uint16_t target_min;
    const uint16_t target_max;
    const uint16_t target_neutral;
    const uint16_t target_offset;
    const uint16_t servo_range_deg;
    ;
};
class ServoMotorDriver {
public:
    ServoMotorDriver(MicroMaestro &maestro, const ServoMotorConfig config);
    ServoMotorDriver() = default;
    ~ServoMotorDriver() = default;

    int setup();

    void rev(bool rev) { reversed_ = rev; }

    void degree(float degree);

private:
    MicroMaestro &maestro_;
    const ServoMotorConfig config_;
    bool reversed_{false};
    float limit_target(uint32_t target) {
        if (target > config_.target_max) {
            return config_.target_max;
        } else if (target < config_.target_min) {
            return config_.target_min;
        }
        return target;
    };
};

}  // namespace servo_motor_driver
