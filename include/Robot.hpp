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
        left_encoder.begin();
        right_encoder.begin();

        beginSensor();

        imu.begin();
        imu.zeroYaw();

        stop();
    }

    void beginSensor(){
        pinMode(front_lidar.xshut_pin, OUTPUT);
        pinMode(left_lidar.xshut_pin, OUTPUT);
        pinMode(right_lidar.xshut_pin, OUTPUT);

        // Switch all sensors off first
        digitalWrite(front_lidar.xshut_pin, LOW);
        digitalWrite(left_lidar.xshut_pin, LOW);
        digitalWrite(right_lidar.xshut_pin, LOW);
        delay(20);

        // Start front and assign 0x30
        digitalWrite(front_lidar.xshut_pin, HIGH);
        delay(10);
        front_lidar.begin();

        // Start left and assign 0x31
        digitalWrite(left_lidar.xshut_pin, HIGH);
        delay(10);
        left_lidar.begin();

        // Start right and assign 0x32
        digitalWrite(right_lidar.xshut_pin, HIGH);
        delay(10);
        right_lidar.begin();
    }


    void startStraightLine(float distance_mm, int16_t pwm = 140) {
        task = TASK_STRAIGHT_LINE;
        finished = false;

        target_distance_mm = distance_mm;
        base_pwm = abs(pwm);

        start_left_rotation = left_encoder.getRotation();
        start_right_rotation = right_encoder.getRotation();

        target_yaw_deg = imu.getYawDeg();

        Serial.print("Straight line target yaw: ");
        Serial.println(target_yaw_deg);
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

    void startTurnHold(float angle_deg) {
        task = TASK_TURN;
        finished = false;

        target_yaw_deg = IMU::wrapAngleDeg(imu.getYawDeg() + angle_deg);

        turn_pid.zeroAndSetTarget(0, 0);

        Serial.print("Turn hold target yaw: ");
        Serial.println(target_yaw_deg);
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

        float current_yaw = imu.getYawDeg();

        // Positive if robot has rotated away from starting heading
        float heading_error = IMU::wrapAngleDeg(current_yaw - target_yaw_deg);

        float Kp_heading = 4.0;
        float correction = Kp_heading * heading_error;

        correction = constrain(correction, -50, 50);

        float left_pwm = base_pwm + correction;
        float right_pwm = base_pwm - correction;

        left_pwm = constrain(left_pwm, 80, 180);
        right_pwm = constrain(right_pwm, 80, 180);

        setDrivePWM(left_pwm, right_pwm);

        Serial.print("Yaw: ");
        Serial.print(current_yaw);
        Serial.print(" Error: ");
        Serial.print(heading_error);
        Serial.print(" Distance: ");
        Serial.print(distance_mm);
        Serial.print(" L_PWM: ");
        Serial.print(left_pwm);
        Serial.print(" R_PWM: ");
        Serial.println(right_pwm);
    }

    void updateWallDistance() {
        uint16_t distance_mm = front_lidar.readDistance();

        Serial.print("Front lidar distance: ");
        Serial.print(distance_mm);
        Serial.println(" mm");

        if (!front_lidar.isReady() || front_lidar.timedOut() || distance_mm == 0) {
            stopMotors();
            Serial.println("Invalid lidar reading. Stopping.");
            return;
        }

        float error_mm = static_cast<float>(distance_mm) - target_wall_distance_mm;

        if (fabs(error_mm) <= wall_tolerance_mm) {
            stopMotors();
            Serial.println("At target wall distance.");
            return;
        }

        float Kp_wall = 1.5;
        float pwm = Kp_wall * error_mm;

        pwm = constrain(pwm, -120, 120);

        if (fabs(pwm) < 65) {
            if (pwm > 0) {
                pwm = 65;
            } else {
                pwm = -65;
            }
        }

        setDrivePWM(pwm, pwm);

        Serial.print("Wall error: ");
        Serial.print(error_mm);
        Serial.print(" PWM: ");
        Serial.println(pwm);
    }

    void updateTurn() {
        float current_yaw = imu.getYawDeg();

        float yaw_error = IMU::wrapAngleDeg(target_yaw_deg - current_yaw);

        Serial.print("Yaw: ");
        Serial.print(current_yaw);
        Serial.print(" Target: ");
        Serial.print(target_yaw_deg);
        Serial.print(" Error: ");
        Serial.println(yaw_error);

        // If close enough, stop motors but KEEP holding this target
        if (fabs(yaw_error) <= turn_tolerance_deg) {
            stopMotors();
            finished = true;
            return;
        }

        finished = false;

        float Kp_turn = 2.5;
        float pwm = Kp_turn * yaw_error;

        pwm = constrain(pwm, -120, 120);

        // Minimum PWM so the robot can actually overcome friction
        if (fabs(pwm) < 65) {
            if (pwm > 0) {
                pwm = 65;
            } else {
                pwm = -65;
            }
        }

        /*
        This sign may need flipping depending on your motor wiring.
        If the robot turns away from the target instead of toward it,
        change this line to setDrivePWM(pwm, -pwm);
        */
        setDrivePWM(-pwm, pwm);
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

        float average_rotation = (fabs(left_rotation) + fabs(right_rotation)) / 2.0;

        return average_rotation * wheel_radius_mm;
    }

    void setDrivePWM(float left_pwm, float right_pwm) {
        left_motor.setPWM(static_cast<int16_t>(constrain(-left_pwm, -255, 255)));

        // Right motor is inverted because negative PWM was forward for your robot
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
