#pragma once

#include <PololuMaestro.h>

namespace servo_motor_driver {

struct ServoMotorConfig {
    ServoMotorConfig(const uint8_t channel, const uint16_t target_neutral, const int offset, const uint16_t target_min,
                     const uint16_t target_max, const float deg_per_target, const bool rev)
        : channel(channel),
          target_neutral(target_neutral),
          offset(offset),
          target_min(target_min),
          target_max(target_max),
          deg_per_target(deg_per_target),
          rev(rev) {}

    const uint8_t channel;
    const uint16_t target_neutral;
    const uint16_t offset;
    const uint16_t target_min;
    const uint16_t target_max;
    const float deg_per_target;
    const bool rev;
};

class ServoMotorDriver {
public:
    ServoMotorDriver(MicroMaestro &maestro, const ServoMotorConfig &config);
    ServoMotorDriver() = default;
    ~ServoMotorDriver() = default;

    int setup();

    void degree(float degree);

    ServoMotorConfig config() const { return config_; }

private:
    MicroMaestro &maestro_;
    const ServoMotorConfig config_;
    int limit_target(int target) {
        if (target > config_.target_max) {
            return config_.target_max;
        } else if (target < config_.target_min) {
            return config_.target_min;
        }
        return target;
    };
};

}  // namespace servo_motor_driver
