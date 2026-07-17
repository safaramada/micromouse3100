#include "Robot.hpp"

#include <math.h>

namespace {

constexpr float MAZE_CELL_DISTANCE_MM = 180.0f;
constexpr float TURN_TOLERANCE_DEG = 2.0f;
constexpr float LIDAR_CENTERING_KP = 0.8f;
constexpr float MAX_LIDAR_CORRECTION = 30.0f;
constexpr uint16_t MIN_SIDE_WALL_MM = 10;
constexpr uint16_t MAX_SIDE_WALL_MM = 140;

bool isForwardCommand(char command) {
    return command == 'f' || command == 'F';
}

bool isLeftCommand(char command) {
    return command == 'l' || command == 'L';
}

bool isRightCommand(char command) {
    return command == 'r' || command == 'R';
}

bool isTurnCommand(char command) {
    return isLeftCommand(command) || isRightCommand(command);
}

}  // namespace

namespace mtrn3100 {

void Robot::startCommandString(const char* commands) {
    task_ = Task::COMMAND_STRING;
    finished_ = false;
    command_string_ = commands;
    command_index_ = 0;
    command_state_ = CommandState::READY;
    stopMotors();
}

void Robot::updateCommandString() {
    if (command_string_ == nullptr) {
        finishTask();
        return;
    }

    switch (command_state_) {
        case CommandState::READY:
            startNextCommand();
            break;
        case CommandState::FORWARD:
            updateForwardCommand();
            break;
        case CommandState::TURN:
            updateTurnCommand();
            break;
    }
}

void Robot::startNextCommand() {
    const char command = command_string_[command_index_];
    if (command == '\0') {
        finishTask();
        return;
    }

    if (isForwardCommand(command)) {
        start_left_rotation_ = left_encoder_.getRotation();
        start_right_rotation_ = right_encoder_.getRotation();
        target_distance_mm_ = MAZE_CELL_DISTANCE_MM;
        target_yaw_deg_ = imu_.getYawDeg();
        command_state_ = CommandState::FORWARD;
        return;
    }

    if (isLeftCommand(command)) {
        target_yaw_deg_ =
            IMU::wrapAngleDeg(imu_.getYawDeg() + 90.0f);
        command_state_ = CommandState::TURN;
        return;
    }

    if (isRightCommand(command)) {
        target_yaw_deg_ =
            IMU::wrapAngleDeg(imu_.getYawDeg() - 90.0f);
        command_state_ = CommandState::TURN;
        return;
    }

    // Ignore unknown characters instead of stopping a whole command sequence.
    ++command_index_;
}

void Robot::updateForwardCommand() {
    if (front_lidar_safety_enabled_) {
        const uint16_t front_distance_mm = front_lidar_.readDistance();
        const bool valid_reading = front_lidar_.isReady() &&
                                   !front_lidar_.timedOut() &&
                                   front_distance_mm > 0;

        if (valid_reading && front_distance_mm <= front_stop_distance_mm_) {
            Serial.print(F("Emergency stop: front wall at "));
            Serial.print(front_distance_mm);
            Serial.println(F(" mm"));

            if (isTurnCommand(command_string_[command_index_ + 1])) {
                Serial.println(F("Skipping forward and starting next turn."));
                completeCurrentCommand();
            } else {
                Serial.println(F("No turn command available. Stopping."));
                finishTask();
            }
            return;
        }
    }

    if (getAverageDistanceMM() >= target_distance_mm_) {
        if (command_string_[command_index_ + 1] == '\0') {
            finishTask();
        } else {
            completeCurrentCommand();
        }
        return;
    }

    const float heading_error =
        IMU::wrapAngleDeg(imu_.getYawDeg() - target_yaw_deg_);
    const float correction = constrain(
        headingCorrection(heading_error) + getLidarCenteringCorrection(),
        -50.0f,
        50.0f);
    const float left_pwm =
        constrain(base_pwm_ + correction, 80.0f, 180.0f);
    const float right_pwm =
        constrain(base_pwm_ - correction, 80.0f, 180.0f);
    setDrivePWM(left_pwm, right_pwm);
}

void Robot::updateTurnCommand() {
    const float yaw_error =
        IMU::wrapAngleDeg(target_yaw_deg_ - imu_.getYawDeg());
    if (fabs(yaw_error) <= TURN_TOLERANCE_DEG) {
        completeCurrentCommand();
        return;
    }

    const float pwm = turnPwm(yaw_error);
    setDrivePWM(-pwm, pwm);
}

void Robot::completeCurrentCommand() {
    stopMotors();
    ++command_index_;
    command_state_ = CommandState::READY;
}

float Robot::getLidarCenteringCorrection() {
    if (!lidar_centering_enabled_) {
        return 0.0f;
    }

    const uint16_t left_distance_mm = left_lidar_.readDistance();
    const uint16_t right_distance_mm = right_lidar_.readDistance();
    const bool left_wall = isValidSideWall(left_lidar_, left_distance_mm);
    const bool right_wall = isValidSideWall(right_lidar_, right_distance_mm);

    float wall_error_mm = 0.0f;
    if (left_wall && right_wall) {
        wall_error_mm =
            static_cast<float>(right_distance_mm) - left_distance_mm;
    } else if (left_wall) {
        wall_error_mm = side_wall_target_mm_ - left_distance_mm;
    } else if (right_wall) {
        wall_error_mm =
            static_cast<float>(right_distance_mm) - side_wall_target_mm_;
    } else {
        return 0.0f;
    }

    return constrain(LIDAR_CENTERING_KP * wall_error_mm,
                     -MAX_LIDAR_CORRECTION,
                     MAX_LIDAR_CORRECTION);
}

bool Robot::isValidSideWall(const Lidar& lidar, uint16_t distance_mm) {
    return lidar.isReady() && !lidar.timedOut() &&
           distance_mm >= MIN_SIDE_WALL_MM &&
           distance_mm <= MAX_SIDE_WALL_MM;
}

}  // namespace mtrn3100
