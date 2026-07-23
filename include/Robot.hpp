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

        beginSensors();

        imu.begin();
        imu.zeroYaw();

        stop();
    }

    void beginSensors(){
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

    void startWallDistance(float front_distance_mm = 100.0f) {
        task = TASK_WALL_DISTANCE;
        finished = false;

        target_wall_distance_mm = front_distance_mm;
        target_yaw_deg = imu.getYawDeg();

        distance_pid.zeroAndSetTarget(0, 0);

        // Reset wall-distance controller state.
        wall_holding_position = false;
        wall_outside_band_start_ms = 0;

        Serial.print("Wall distance target: ");
        Serial.print(target_wall_distance_mm);
        Serial.println(" mm");

        Serial.print("Wall distance target yaw: ");
        Serial.println(target_yaw_deg);
    }

    void startTurn(float angle_deg) {
        task = TASK_TURN;
        finished = false;
        turn_hold_enabled = false;
        target_yaw_deg = IMU::wrapAngleDeg(imu.getYawDeg() + angle_deg);
        turn_pid.zeroAndSetTarget(0, 0);
    }

    void startCommandString(const char commands[]) {
        task = TASK_COMMAND_STRING;
        finished = false;
        command_string = commands;
        command_index = 0;
        command_state = COMMAND_READY;

        stopMotors();
    }

    void enableLidarCentering(bool enabled, float target_side_distance_mm = 45.0) {
        lidar_centering_enabled = enabled;
        side_wall_target_mm = target_side_distance_mm;
    }

    void enableFrontLidarSafety(bool enabled, uint16_t stop_distance_mm = 40) {
        front_lidar_safety_enabled = enabled;
        front_stop_distance_mm = stop_distance_mm;
    }

    void startTurnHold(float angle_deg) {
        task = TASK_TURN;
        finished = false;
        turn_hold_enabled = true;

        // Save this once. Do not recalculate it after the robot is disturbed.
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

        bool valid_reading =
            front_lidar.isReady() &&
            !front_lidar.timedOut() &&
            distance_mm > 0;

        if (!valid_reading) {
            stopMotors();
            wall_outside_band_start_ms = 0;

            Serial.println("Invalid lidar reading. Stopping.");
            return;
        }

        float error_mm =
            static_cast<float>(distance_mm) -
            target_wall_distance_mm;

        /*
        * HOLDING STATE
        *
        * Once the robot reaches the target, keep the motors stopped.
        * Small LiDAR changes will not make it immediately move again.
        */
        if (wall_holding_position) {
            stopMotors();

            if (fabs(error_mm) > wall_restart_tolerance_mm) {
                // Start timing when the reading first leaves the larger band.
                if (wall_outside_band_start_ms == 0) {
                    wall_outside_band_start_ms = millis();
                }

                // Only restart if the error persists.
                if (millis() - wall_outside_band_start_ms >=
                    wall_restart_delay_ms) {

                    wall_holding_position = false;
                    wall_outside_band_start_ms = 0;

                    distance_pid.zeroAndSetTarget(0, 0);

                    Serial.println(
                        "Wall moved. Restarting distance controller."
                    );
                }
            } else {
                // It returned inside the restart band, so it was likely noise.
                wall_outside_band_start_ms = 0;
            }

            return;
        }

        /*
        * MOVING STATE
        *
        * Enter the holding state when accurately positioned.
        */
        if (fabs(error_mm) <= wall_stop_tolerance_mm) {
            stopMotors();

            wall_holding_position = true;
            wall_outside_band_start_ms = 0;

            distance_pid.zeroAndSetTarget(0, 0);

            Serial.println("At target wall distance. Holding position.");
            return;
        }

        float Kp_wall = 1.5f;
        float pwm = Kp_wall * error_mm;

        pwm = constrain(pwm, -120.0f, 120.0f);

        // Minimum PWM needed to overcome motor friction.
        if (fabs(pwm) < 65.0f) {
            pwm = (pwm > 0.0f) ? 65.0f : -65.0f;
        }

        // Heading correction to keep the robot straight.
        float current_yaw = imu.getYawDeg();

        float heading_error =
            IMU::wrapAngleDeg(current_yaw - target_yaw_deg);

        float Kp_heading = 4.0f;
        float correction = Kp_heading * heading_error;

        correction = constrain(correction, -50.0f, 50.0f);

        float left_pwm =
            constrain(pwm + correction, -180.0f, 180.0f);

        float right_pwm =
            constrain(pwm - correction, -180.0f, 180.0f);

        setDrivePWM(left_pwm, right_pwm);

        Serial.print("Wall error: ");
        Serial.print(error_mm);

        Serial.print(" Holding: ");
        Serial.print(wall_holding_position);

        Serial.print(" Yaw: ");
        Serial.print(current_yaw);

        Serial.print(" Heading error: ");
        Serial.print(heading_error);

        Serial.print(" L_PWM: ");
        Serial.print(left_pwm);

        Serial.print(" R_PWM: ");
        Serial.println(right_pwm);
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

        // Stop at the target. In hold mode TASK_TURN remains active, so a
        // later pickup/release disturbance causes this controller to run again.
        if (fabs(yaw_error) <= turn_tolerance_deg) {
            stopMotors();
            finished = true;

            if (!turn_hold_enabled) {
                finishTask();
            }

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

    void startNextCommand() {

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

    void updateForwardCommand() {
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

                char next_command = command_string[command_index +1];

                if (next_command =='l' || next_command == 'L' ||
                    next_command =='r' || next_command == 'R' ) {

                        Serial.print("Skipping forward and starting next turn");

                        completeCurrentCommand();
                        return;
                    }

                Serial.print("No turn command available. Stopping.");
                finishTask();
                return;
            }
        }

        // Stop normally after travelling one maze cell
        if (distance_mm >= target_distance_mm) {

            if (command_string[command_index +1] == '\0') {
                finishTask();
                return;
            }
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

    float getLidarCenteringCorrection() {
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

    bool isValidSideWall(Lidar& lidar, uint16_t distance_mm) {
        return lidar.isReady() &&
               !lidar.timedOut() &&
               distance_mm >= min_side_wall_mm &&
               distance_mm <= max_side_wall_mm;
    }

    void updateTurnCommand() {
        float yaw_error =
            IMU::wrapAngleDeg(target_yaw_deg - imu.getYawDeg());

        if (fabs(yaw_error) <= turn_tolerance_deg) {
            completeCurrentCommand();
            return;
        }

        // AI-assisted change: match the proven Task 3 turning controller.
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

    void completeCurrentCommand() {
        stopMotors();

        command_index++;
        command_state = COMMAND_READY;
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
    bool turn_hold_enabled = false;

    int16_t base_pwm = 120;
    float target_distance_mm = 0;
    // float target_wall_distance_mm = 100;
    float target_yaw_deg = 0;
    float start_left_rotation = 0;
    float start_right_rotation = 0;
    // float wall_tolerance_mm = 5;
    float turn_tolerance_deg = 2;

    // 23/07 attempt to change jittery porblem 
    float target_wall_distance_mm = 100;

    // Enter the resting state within ±4 mm.
    const float wall_stop_tolerance_mm = 4.0f;

    // Do not move again until the error exceeds ±8 mm.
    const float wall_restart_tolerance_mm = 8.0f;

    // The error must remain outside ±8 mm for this duration.
    const unsigned long wall_restart_delay_ms = 150;

    ////////

    bool wall_holding_position = false;
    unsigned long wall_outside_band_start_ms = 0;

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
