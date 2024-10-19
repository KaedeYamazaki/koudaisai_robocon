
#include "robot_controller.hpp"

namespace robot {
RobotController::RobotController(const std::string &controller_mac, robot::Robot &robot)
    : controller_mac_(controller_mac), robot_(robot), state_(RobotState::setup), bucket_value_count_(0) {};

int RobotController::setup() {
    int i = 0;
    i += robot_.setup();
    i += PS4.begin(controller_mac_.c_str());
    return i;
}

void RobotController::cycle() {
    if (PS4.isConnected()) {
        if (PS4.Options()) {
            Serial.println("set standby mode");
            state_ == RobotState::standby;
            return;
        }
        if (state_ == RobotState::setup) {
            state_ == RobotState::standby;
            Serial.println("setup to standby");
            return;
        } else if (state_ == RobotState::standby) {
            Serial.println("stand by mode");
            if (PS4.Square()) {
                Serial.println("set semi auto mode");
                state_ = RobotState::semi_auto;
                return;
            }
            if (PS4.Triangle()) {
                Serial.println("set manual mode");
                state_ = RobotState::manual;
                return;
            }
        }
        if (state_ == RobotState::semi_auto) {
            semi_auto();
            return;
        }
        if (state_ == RobotState::manual) {
            manual();
            return;
        }

    } else {
        Serial.println("controller not connected");
    }
}

void RobotController::semi_auto() {
    drive();

    bool L2 = PS4.L2();
    bool R2 = PS4.R2();

    if (L2) { robot_.arm_close(); }
    if (R2) { robot_.arm_open(); }

    bool left = PS4.Left();
    bool up = PS4.Up();
    bool down = PS4.Down();

    if (left) { robot_.bucket_lift(); }
    if (down) { robot_.bucket_scoop(); }
    if (up) { robot_.bucket_turn_over(); }
}

void RobotController::manual() {
    drive();

    auto L2 = PS4.L2Value();
    auto R2 = PS4.R2Value();

    float left_arm_angle = 270.f * static_cast<float>(L2 / INT8_MAX);
    float right_arm_angle = 270.f * static_cast<float>(R2 / INT8_MAX);

    robot_.arm_degree(left_arm_angle, right_arm_angle);

    bool left = PS4.Left();
    bool up = PS4.Up();
    bool down = PS4.Down();

    if (left) { robot_.bucket_degree(0.f); }
    if (down) {
        bucket_value_count_ -= 10;
        robot_.bucket_degree(static_cast<float>(bucket_value_count_));
    }
    if (up) {
        bucket_value_count_ += 10;
        robot_.bucket_degree(static_cast<float>(bucket_value_count_));
    }
}

void RobotController::drive() {
    int L = PS4.LStickY();
    int R = PS4.RStickY();

    if (abs(L) <= 15) L = 0;
    if (abs(R) <= 15) R = 0;

    float left_duty;
    float right_duty;

    left_duty = static_cast<float>(L / INT8_MAX);
    right_duty = static_cast<float>(R / INT8_MAX);

    robot_.drive(left_duty, right_duty);
}

}  // namespace robot
