#include "Robot.hpp"

namespace mtrn3100 {

void Robot::startCommandString(const char commands[]) {
    task = TASK_COMMAND_STRING;
    finished = false;
    command_string = commands;
    command_index = 0;
    command_state = COMMAND_READY;

    stopMotors();
}

void Robot::enableLidarCentering(bool enabled, float target_side_distance_mm) {
    lidar_centering_enabled = enabled;
    side_wall_target_mm = target_side_distance_mm;
}

void Robot::enableFrontLidarSafety(bool enabled, uint16_t stop_distance_mm) {
    front_lidar_safety_enabled = enabled;
    front_stop_distance_mm = stop_distance_mm;
}

void Robot::updateCommandString() {
    if (command_string == nullptr) {
        finishTask();
        return;
    }

    if (command_state == COMMAND_READY) {
        startNextCommand();
        return;
    }

    if (command_state == COMMAND_FORWARD) {
        updateForwardCommand();
        return;
    }

    if (command_state == COMMAND_TURN) {
        updateTurnCommand();
    }
}

void Robot::startNextCommand() {
    char command = command_string[command_index];

    if (command == '\0') {
        finishTask();
        return;
    }

    if (command == 'f' || command == 'F') {
        start_left_rotation = left_encoder.getRotation();
        start_right_rotation = right_encoder.getRotation();

        target_distance_mm = maze_cell_distance_mm;
        target_yaw_deg = imu.getYawDeg();

        heading_pid.zeroAndSetTarget(0, 0);
        command_state = COMMAND_FORWARD;
    }

    else if (command == 'l' || command == 'L') {
        target_yaw_deg =
            IMU::wrapAngleDeg(imu.getYawDeg() + 90.0);

        turn_pid.zeroAndSetTarget(0, 0);
        command_state = COMMAND_TURN;
    }

    else if (command == 'r' || command == 'R') {
        target_yaw_deg =
            IMU::wrapAngleDeg(imu.getYawDeg() - 90.0);

        turn_pid.zeroAndSetTarget(0, 0);
        command_state = COMMAND_TURN;
    }
    else {
        command_index++;
    }
}

void Robot::updateForwardCommand() {
    float distance_mm = getAverageDistanceMM();

    // Front LiDAR collision check
    if (front_lidar_safety_enabled) {
        uint16_t front_distance_mm = front_lidar.readDistance();

        bool valid_front_reading =
            front_lidar.isReady() &&
            !front_lidar.timedOut() &&
            front_distance_mm > 0;

        if (valid_front_reading &&
            front_distance_mm <= front_stop_distance_mm) {

            Serial.print("Emergency stop: front wall at ");
            Serial.print(front_distance_mm);
            Serial.println(" mm");

            finishTask();
            return;
        }
    }

    // Stop normally after travelling one maze cell
    if (distance_mm >= target_distance_mm) {
        completeCurrentCommand();
        return;
    }

    float heading_error =
        IMU::wrapAngleDeg(imu.getYawDeg() - target_yaw_deg);

    float imu_correction = 4.0 * heading_error;
    float lidar_correction = getLidarCenteringCorrection();

    float correction =
        imu_correction + lidar_correction;

    correction = constrain(correction, -50, 50);

    float left_pwm =
        constrain(base_pwm + correction, 80, 180);

    float right_pwm =
        constrain(base_pwm - correction, 80, 180);

    setDrivePWM(left_pwm, right_pwm);
}

float Robot::getLidarCenteringCorrection() {
    if (!lidar_centering_enabled) {
        return 0;
    }

    uint16_t left_distance_mm = left_lidar.readDistance();
    bool left_wall = isValidSideWall(left_lidar, left_distance_mm);

    uint16_t right_distance_mm = right_lidar.readDistance();
    bool right_wall = isValidSideWall(right_lidar, right_distance_mm);

    float wall_error_mm = 0;

    if (left_wall && right_wall) {
        wall_error_mm = static_cast<float>(right_distance_mm) - left_distance_mm;
    } else if (left_wall) {
        wall_error_mm = side_wall_target_mm - left_distance_mm;
    } else if (right_wall) {
        wall_error_mm = static_cast<float>(right_distance_mm) - side_wall_target_mm;
    } else {
        return 0;
    }

    float correction = lidar_centering_kp * wall_error_mm;
    return constrain(correction, -max_lidar_correction, max_lidar_correction);
}

bool Robot::isValidSideWall(Lidar& lidar, uint16_t distance_mm) {
    return lidar.isReady() &&
           !lidar.timedOut() &&
           distance_mm >= min_side_wall_mm &&
           distance_mm <= max_side_wall_mm;
}

void Robot::updateTurnCommand() {
    float yaw_error =
        IMU::wrapAngleDeg(target_yaw_deg - imu.getYawDeg());

    if (fabs(yaw_error) <= turn_tolerance_deg) {
        completeCurrentCommand();
        return;
    }

    // AI-assisted change: match the proven Part 3 turning controller.
    float Kp_turn = 2.5;
    float pwm = Kp_turn * yaw_error;

    pwm = constrain(pwm, -120, 120);

    if (fabs(pwm) < 65) {
        if (pwm > 0) {
            pwm = 65;
        } else {
            pwm = -65;
        }
    }

    setDrivePWM(-pwm, pwm);
}

void Robot::completeCurrentCommand() {
    stopMotors();

    command_index++;
    command_state = COMMAND_READY;
}

}  // namespace mtrn3100
