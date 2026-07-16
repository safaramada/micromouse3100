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

enum CommandState {
    COMMAND_READY,
    COMMAND_FORWARD,
    COMMAND_TURN
};

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
          float wheel_base_mm = 80.0);

    void begin();
    void beginSensors();

    // Part 1: straight-line driving
    void startStraightLine(float distance_mm, int16_t pwm = 140);

    // Part 2: front-wall distance control
    void startWallDistance(float front_distance_mm);

    // Part 3: turning
    void startTurn(float angle_deg);

    // Part 4: command-string execution
    void startCommandString(const char commands[]);
    void enableLidarCentering(bool enabled, float target_side_distance_mm = 45.0);
    void enableFrontLidarSafety(bool enabled, uint16_t stop_distance_mm = 40);

    void update();
    bool isFinished();
    void stop();

private:
    // Part 1
    void updateStraightLine();

    // Part 2
    void updateWallDistance();

    // Part 3
    void updateTurn();

    // Part 4
    void updateCommandString();
    void startNextCommand();
    void updateForwardCommand();
    float getLidarCenteringCorrection();
    bool isValidSideWall(Lidar& lidar, uint16_t distance_mm);
    void updateTurnCommand();
    void completeCurrentCommand();

    // Helpers shared by more than one task
    float getAverageDistanceMM();
    void setDrivePWM(float left_pwm, float right_pwm);
    void stopMotors();
    void finishTask();

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
    float turn_tolerance_deg = 2;

    bool lidar_centering_enabled = false;
    float side_wall_target_mm = 45.0;
    const float lidar_centering_kp = 0.8;
    const float max_lidar_correction = 30.0;
    const uint16_t min_side_wall_mm = 10;
    const uint16_t max_side_wall_mm = 140;
    bool front_lidar_safety_enabled = false;
    uint16_t front_stop_distance_mm = 40;

    const char* command_string = nullptr;
    uint8_t command_index = 0;
    CommandState command_state = COMMAND_READY;
    const float maze_cell_distance_mm = 180.0;
};

}  // namespace mtrn3100
