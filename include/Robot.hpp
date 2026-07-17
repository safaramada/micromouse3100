#pragma once

#include <Arduino.h>

#include "Encoder.hpp"
#include "IMU.hpp"
#include "Lidar.hpp"
#include "Motor.hpp"

namespace mtrn3100 {

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
          float wheel_radius_mm = 16.0f);

    void begin();
    void update();
    void stop();

    void startStraightLine(float distance_mm, int16_t pwm = 140);
    void startWallDistance(float front_distance_mm);
    void startTurn(float angle_deg);
    void startTurnHold(float angle_deg);
    void startCommandString(const char* commands);

    void enableLidarCentering(bool enabled,
                              float target_side_distance_mm = 45.0f);
    void enableFrontLidarSafety(bool enabled,
                                uint16_t stop_distance_mm = 40);

    bool isFinished() const;

private:
    enum class Task : uint8_t {
        IDLE,
        STRAIGHT_LINE,
        WALL_DISTANCE,
        TURN,
        COMMAND_STRING
    };

    enum class CommandState : uint8_t {
        READY,
        FORWARD,
        TURN
    };

    void beginSensors();
    void updateStraightLine();
    void updateWallDistance();
    void updateTurn();

    void updateCommandString();
    void startNextCommand();
    void updateForwardCommand();
    void updateTurnCommand();
    void completeCurrentCommand();
    float getLidarCenteringCorrection();
    static bool isValidSideWall(const Lidar& lidar, uint16_t distance_mm);

    float getAverageDistanceMM() const;
    static float headingCorrection(float heading_error_deg);
    static float turnPwm(float yaw_error_deg);
    void setDrivePWM(float left_pwm, float right_pwm);
    void stopMotors();
    void finishTask();

    Motor& left_motor_;
    Motor& right_motor_;
    Encoder& left_encoder_;
    Encoder& right_encoder_;
    Lidar& front_lidar_;
    Lidar& left_lidar_;
    Lidar& right_lidar_;
    IMU& imu_;

    const float wheel_radius_mm_;
    float target_distance_mm_ = 0.0f;
    float target_wall_distance_mm_ = 100.0f;
    float target_yaw_deg_ = 0.0f;
    float start_left_rotation_ = 0.0f;
    float start_right_rotation_ = 0.0f;
    float side_wall_target_mm_ = 45.0f;

    const char* command_string_ = nullptr;
    int16_t base_pwm_ = 120;
    uint16_t front_stop_distance_mm_ = 40;
    Task task_ = Task::IDLE;
    CommandState command_state_ = CommandState::READY;
    uint8_t command_index_ = 0;
    bool finished_ = true;
    bool turn_hold_enabled_ = false;
    bool lidar_centering_enabled_ = false;
    bool front_lidar_safety_enabled_ = false;
};

}  // namespace mtrn3100
