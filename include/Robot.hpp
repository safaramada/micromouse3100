#pragma once

#include <Arduino.h>
#include <math.h>

#include "Encoder.hpp"
#include "IMU.hpp"
#include "Lidar.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"

namespace mtrn3100 {


enum RobotTask {
    TASK_IDLE,
    TASK_STRAIGHT_LINE,
    TASK_WALL_DISTANCE,
    TASK_TURN,
    TASK_COMMAND_STRING
};

// The robot class combines the low-level hardware classes into Task 3 actions.
// It is intentionally a starter structure so each controller can be tuned in one place.
class Robot {
public:
    Robot(Motor& left_motor,
          Motor& right_motor,
          Encoder& left_encoder,
          Encoder& right_encoder,
          Lidar& front_lidar,
          Lidar& left_lidar,
          Lidar& right_lidar,
          IMU& imu,
          float wheel_radius_mm = 16.0,
          float wheel_base_mm = 80.0)
        : left_motor(left_motor),
          right_motor(right_motor),
          left_encoder(left_encoder),
          right_encoder(right_encoder),
          front_lidar(front_lidar),
          left_lidar(left_lidar),
          right_lidar(right_lidar),
          imu(imu),
          wheel_radius_mm(wheel_radius_mm),
          wheel_base_mm(wheel_base_mm),
          heading_pid(4.0, 0.0, 0.0),
          distance_pid(2.0, 0.0, 0.0),
          turn_pid(4.0, 0.0, 0.0) {}

    void begin() {
        front_lidar.begin();
        imu.begin();
        imu.zeroYaw();
        stop();
    }

    void startStraightLine(float distance_mm, int16_t pwm = 120) {
        task = TASK_STRAIGHT_LINE;
        finished = false;
        target_distance_mm = distance_mm;
        base_pwm = pwm;
        start_left_rotation = left_encoder.getRotation();
        start_right_rotation = right_encoder.getRotation();
        target_yaw_deg = imu.getYawDeg();
        heading_pid.zeroAndSetTarget(0, 0);
    }

    void startWallDistance(float front_distance_mm) {
        task = TASK_WALL_DISTANCE;
        finished = false;
        target_wall_distance_mm = front_distance_mm;
        distance_pid.zeroAndSetTarget(0, 0);
    }

    void startTurn(float angle_deg) {
        task = TASK_TURN;
        finished = false;
        target_yaw_deg = IMU::wrapAngleDeg(imu.getYawDeg() + angle_deg);
        turn_pid.zeroAndSetTarget(0, 0);
    }

    void startCommandString(const char commands[]) {
        task = TASK_COMMAND_STRING;
        finished = false;
        command_string = commands;
        command_index = 0;

        // TODO: Start the first command and advance one command at a time.
        // Forward should drive one maze cell. Left/right should turn 90 degrees.
    }

    void update() {
        imu.update();

        switch (task) {
            case TASK_STRAIGHT_LINE:
                updateStraightLine();
                break;

            case TASK_WALL_DISTANCE:
                updateWallDistance();
                break;

            case TASK_TURN:
                updateTurn();
                break;

            case TASK_COMMAND_STRING:
                updateCommandString();
                break;

            case TASK_IDLE:
            default:
                stopMotors();
                break;
        }
    }

    bool isFinished() {
        return finished;
    }

    void stop() {
        stopMotors();
        task = TASK_IDLE;
        finished = true;
    }

private:
    void updateStraightLine() {
        float distance_mm = getAverageDistanceMM();

        if (distance_mm >= target_distance_mm) {
            finishTask();
            return;
        }

        float heading_error = IMU::wrapAngleDeg(target_yaw_deg - imu.getYawDeg());
        float correction = heading_pid.compute(-heading_error);
        setDrivePWM(base_pwm - correction, base_pwm + correction);
    }

    void updateWallDistance() {
        uint16_t distance_mm = front_lidar.readDistance();
        float error_mm = static_cast<float>(distance_mm) - target_wall_distance_mm;

        if (fabs(error_mm) <= wall_tolerance_mm) {
            stopMotors();
            return;
        }

        float pwm = distance_pid.compute(-error_mm);
        setDrivePWM(pwm, pwm);
    }

    void updateTurn() {
        float yaw_error = IMU::wrapAngleDeg(target_yaw_deg - imu.getYawDeg());

        if (fabs(yaw_error) <= turn_tolerance_deg) {
            stopMotors();
            return;
        }

        float pwm = turn_pid.compute(-yaw_error);
        setDrivePWM(pwm, -pwm);
    }

    void updateCommandString() {
        if (command_string == nullptr || command_string[command_index] == '\0') {
            finishTask();
            return;
        }

        // TODO: Replace this placeholder with a command state machine.
        // command_string[command_index] will be 'f', 'l', or 'r'.
        finishTask();
    }

    float getAverageDistanceMM() {
        float left_rotation = left_encoder.getRotation() - start_left_rotation;
        float right_rotation = right_encoder.getRotation() - start_right_rotation;
        float average_rotation = (left_rotation + right_rotation) / 2.0;

        return average_rotation * wheel_radius_mm;
    }

    void setDrivePWM(float left_pwm, float right_pwm) {
        left_motor.setPWM(static_cast<int16_t>(constrain(left_pwm, -255, 255)));
        right_motor.setPWM(static_cast<int16_t>(constrain(right_pwm, -255, 255)));
    }

    void stopMotors() {
        left_motor.setPWM(0);
        right_motor.setPWM(0);
    }

    void finishTask() {
        stop();
        finished = true;
    }

private:
    Motor& left_motor;
    Motor& right_motor;
    Encoder& left_encoder;
    Encoder& right_encoder;
    Lidar& front_lidar;
    Lidar& left_lidar;
    Lidar& right_lidar;
    IMU& imu;

    const float wheel_radius_mm;
    const float wheel_base_mm;

    PIDController heading_pid;
    PIDController distance_pid;
    PIDController turn_pid;

    RobotTask task = TASK_IDLE;
    bool finished = true;

    int16_t base_pwm = 120;
    float target_distance_mm = 0;
    float target_wall_distance_mm = 100;
    float target_yaw_deg = 0;
    float start_left_rotation = 0;
    float start_right_rotation = 0;
    float wall_tolerance_mm = 5;
    float turn_tolerance_deg = 5;

    const char* command_string = nullptr;
    uint8_t command_index = 0;
};

}  // namespace mtrn3100
