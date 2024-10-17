#include "servo_motor_driver.hpp"

namespace servo_motor_driver {
ServoMotorDriver::ServoMotorDriver(MicroMaestro &maestro, const ServoMotorConfig config)
    : maestro_(maestro), config_(config) {}

int ServoMotorDriver::setup() {
    maestro_.setAcceleration(config_.channel, 0);
    maestro_.setSpeed(config_.channel, 0);
    maestro_.setTarget(config_.channel, config_.target_neutral);
    return 1;
}

void ServoMotorDriver::degree(float degree) {
    if (reversed_) { degree = -degree; }

    uint32_t target = static_cast<uint32_t>(
        config_.target_neutral + degree * (config_.target_max - config_.target_min) / config_.servo_range_deg);
    target = limit_target(target);
    
    maestro_.setTarget(config_.channel, target);
}
}  // namespace servo_motor_driver