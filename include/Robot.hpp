#pragma once

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>

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
    COMMAND_TURN,
    COMMAND_CUSTOM_TURN,
    COMMAND_CUSTOM_FORWARD
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
          heading_pid(2.0, 0.0, 0.06),
          distance_pid(1.2, 0.02, 0.04),
          turn_pid(1.45, 0.0, 0.12) {}

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


    // void startStraightLine(float distance_mm, int16_t pwm = 130) {
    //     task = TASK_STRAIGHT_LINE;
    //     finished = false;

    //     target_distance_mm = distance_mm;
    //     base_pwm = constrain(abs(pwm), 80, 180);

    //     start_left_rotation = left_encoder.getRotation();
    //     start_right_rotation = right_encoder.getRotation();

    //     target_yaw_deg = imu.getYawDeg();
    //     heading_pid.reset();
    //     resetSideLidarAvoidanceState();

    //     Serial.print("Straight line target yaw: ");
    //     Serial.println(target_yaw_deg);
    // }

    // void startWallDistance(float front_distance_mm = 100.0f) {
    //     task = TASK_WALL_DISTANCE;
    //     finished = false;

    //     target_wall_distance_mm = front_distance_mm;
    //     target_yaw_deg = imu.getYawDeg();

    //     distance_pid.reset();
    //     heading_pid.reset();

    //     // Reset wall-distance controller state.
    //     wall_holding_position = false;
    //     wall_outside_band_start_ms = 0;

    //     Serial.print("Wall distance target: ");
    //     Serial.print(target_wall_distance_mm);
    //     Serial.println(" mm");

    //     Serial.print("Wall distance target yaw: ");
    //     Serial.println(target_yaw_deg);
    // }

    // void startTurn(float angle_deg) {
    //     task = TASK_TURN;
    //     finished = false;
    //     turn_hold_enabled = false;
    //     target_yaw_deg = IMU::wrapAngleDeg(imu.getYawDeg() + angle_deg);
    //     beginTurnController();
    // }

    void startCommandString(const char commands[], int16_t pwm = 130) {
        task = TASK_COMMAND_STRING;
        finished = false;
        command_string = commands;
        command_index = 0;
        command_state = COMMAND_READY;
        active_forward_command_count = 1;
        command_pause_active = false;
        base_pwm = constrain(abs(pwm), 80, 180);

        stopMotors();
        resetSideLidarAvoidanceState();
    }

    void enableSideLidarAvoidance(bool enabled,
                                  float minimum_clearance_mm = 50.0f) {
        side_lidar_avoidance_enabled = enabled;
        side_clearance_mm = minimum_clearance_mm;
        resetSideLidarAvoidanceState();
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

        beginTurnController();

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

        float imu_correction = heading_pid.computeFromError(heading_error);
        float lidar_correction = getSideLidarAvoidanceCorrection();

        // If a side wall is too close, speed up the wheel nearest that wall
        // and slow the opposite wheel to steer away from it.
        float correction = imu_correction + lidar_correction;

        correction = constrain(
            correction,
            -max_forward_correction,
            max_forward_correction
        );

        float left_pwm = constrain(base_pwm + correction, 80, 180);
        float right_pwm = constrain(base_pwm - correction, 80, 180);

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

        bool valid_reading = front_lidar.isReadingValid();

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

                    distance_pid.reset(error_mm);

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

            distance_pid.reset();

            Serial.println("At target wall distance. Holding position.");
            return;
        }

        float pwm = distance_pid.computeFromError(error_mm);

        pwm = constrain(pwm, -120.0f, 120.0f);

        // Minimum PWM needed to overcome motor friction.
        if (fabs(pwm) < 65.0f) {
            pwm = (pwm > 0.0f) ? 65.0f : -65.0f;
        }

        // Heading correction to keep the robot straight.
        float current_yaw = imu.getYawDeg();

        float heading_error =
            IMU::wrapAngleDeg(current_yaw - target_yaw_deg);

        float correction = heading_pid.computeFromError(heading_error);

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

            if (turn_settle_start_ms == 0) {
                turn_settle_start_ms = millis();
            }

            if (millis() - turn_settle_start_ms < turn_settle_time_ms) {
                finished = false;
                return;
            }

            finished = true;
            if (!turn_hold_enabled) {
                finishTask();
            }

            return;
        }

        turn_settle_start_ms = 0;
        finished = false;

        float pwm = getTurnPWM(yaw_error);

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
            if (command_pause_active) {
                if (millis() - command_pause_start_ms < command_pause_time_ms) {
                    return;
                }

                command_pause_active = false;
            }

            startNextCommand();
            return;
        }

        if (command_state == COMMAND_FORWARD) {
            updateForwardCommand();
            return;
        }

        if (command_state == COMMAND_TURN) {
            updateTurnCommand();
            return;
        }

        if (command_state == COMMAND_CUSTOM_TURN) {
            updateCustomTurn();
            return;
        }

        if (command_state == COMMAND_CUSTOM_FORWARD) {
            updateCustomForward();
        }
    }

    void startNextCommand() {

        char command = command_string[command_index];

        if (command == '\0') {
            finishTask();
            return;
        }

        if (command == 'f' || command == 'F') {
            active_forward_command_count = 1;

            // Run consecutive forward cells as one continuous movement. This
            // avoids adding a separate stop/overshoot error at every cell.
            while (command_string[command_index + active_forward_command_count] == 'f' ||
                   command_string[command_index + active_forward_command_count] == 'F') {
                active_forward_command_count++;
            }

            start_left_rotation = left_encoder.getRotation();
            start_right_rotation = right_encoder.getRotation();

            target_distance_mm =
                maze_cell_distance_mm * active_forward_command_count;
            target_yaw_deg = imu.getYawDeg();

            heading_pid.reset();
            resetSideLidarAvoidanceState();
            command_state = COMMAND_FORWARD;
        }

        else if (command == 'l' || command == 'L') {
            target_yaw_deg =
                IMU::wrapAngleDeg(imu.getYawDeg() + 90.0);

            beginTurnController();
            command_state = COMMAND_TURN;
        }

        else if (command == 'r' || command == 'R') {
            target_yaw_deg =
                IMU::wrapAngleDeg(imu.getYawDeg() - 90.0);

            beginTurnController();
            command_state = COMMAND_TURN;
        }
        else if (command == '(') {
            parseCustomCommand();
        }
        else {

            command_index++;

        }

    }

    void parseCustomCommand() {
        // Format: (angle_deg,distance_mm). Positive angles turn left and
        // negative angles turn right, matching the existing L/R commands.
        const char* angle_start = command_string + command_index + 1;
        char* angle_end = nullptr;
        double parsed_angle = strtod(angle_start, &angle_end);

        if (angle_end == angle_start || !isfinite(parsed_angle)) {
            Serial.println(F("Invalid custom command: invalid angle"));
            finishTask();
            return;
        }

        while (*angle_end == ' ' || *angle_end == '\t') {
            angle_end++;
        }

        if (*angle_end != ',') {
            Serial.println(F("Invalid custom command: missing comma"));
            finishTask();
            return;
        }

        const char* distance_start = angle_end + 1;
        char* distance_end = nullptr;
        double parsed_distance = strtod(distance_start, &distance_end);

        if (distance_end == distance_start || !isfinite(parsed_distance)) {
            Serial.println(F("Invalid custom command: invalid distance"));
            finishTask();
            return;
        }

        while (*distance_end == ' ' || *distance_end == '\t') {
            distance_end++;
        }

        if (*distance_end != ')') {
            Serial.println(F("Invalid custom command: missing closing )"));
            finishTask();
            return;
        }

        if (parsed_angle < -180.0 || parsed_angle > 180.0) {
            Serial.println(F("Invalid custom command: angle must be -180 to 180"));
            finishTask();
            return;
        }

        if (parsed_distance < 0.0) {
            Serial.println(F("Invalid custom command: distance must be non-negative"));
            finishTask();
            return;
        }

        custom_angle_deg = static_cast<float>(parsed_angle);
        custom_distance_mm = static_cast<float>(parsed_distance);
        custom_command_end_index = static_cast<uint16_t>(
            distance_end - command_string + 1
        );

        Serial.print(F("Custom command: turn "));
        Serial.print(custom_angle_deg);
        Serial.print(F(" deg, drive "));
        Serial.print(custom_distance_mm);
        Serial.println(F(" mm"));

        if (fabs(custom_angle_deg) > custom_angle_tolerance_deg) {
            target_yaw_deg = IMU::wrapAngleDeg(
                imu.getYawDeg() + custom_angle_deg
            );

            beginTurnController();
            command_state = COMMAND_CUSTOM_TURN;
            return;
        }

        if (custom_distance_mm > 0.0f) {
            startCustomForward();
            return;
        }

        completeCustomCommand();
    }

    void startCustomForward() {
        start_left_rotation = left_encoder.getRotation();
        start_right_rotation = right_encoder.getRotation();

        target_distance_mm = custom_distance_mm;
        target_yaw_deg = imu.getYawDeg();

        heading_pid.reset();
        resetSideLidarAvoidanceState();
        command_state = COMMAND_CUSTOM_FORWARD;
    }

    void updateCustomTurn() {
        float yaw_error =
            IMU::wrapAngleDeg(target_yaw_deg - imu.getYawDeg());

        if (fabs(yaw_error) <= turn_tolerance_deg) {
            stopMotors();

            if (turn_settle_start_ms == 0) {
                turn_settle_start_ms = millis();
            }

            if (millis() - turn_settle_start_ms >= turn_settle_time_ms) {
                turn_settle_start_ms = 0;

                if (custom_distance_mm > 0.0f) {
                    startCustomForward();
                } else {
                    completeCustomCommand();
                }
            }

            return;
        }

        turn_settle_start_ms = 0;
        float pwm = getTurnPWM(yaw_error);
        setDrivePWM(-pwm, pwm);
    }

    void updateCustomForward() {
        float distance_mm = getAverageDistanceMM();

        if (finishCustomForwardIfNeeded(distance_mm)) {
            return;
        }

        if (front_lidar_safety_enabled) {
            uint16_t front_distance_mm = front_lidar.readDistance();

            if (front_lidar.isReadingValid() &&
                front_distance_mm <= front_stop_distance_mm) {
                Serial.println(F("Emergency stop during custom forward"));
                finishTask();
                return;
            }

            // The LiDAR call blocks, so check the encoders again before
            // issuing another motor command.
            distance_mm = getAverageDistanceMM();
            if (finishCustomForwardIfNeeded(distance_mm)) {
                return;
            }
        }

        imu.update();
        float heading_error =
            IMU::wrapAngleDeg(imu.getYawDeg() - target_yaw_deg);
        float imu_correction = heading_pid.computeFromError(heading_error);

        // A custom (angle,distance) command is intended to follow a precise
        // straight heading. Keep front collision stopping above, but do not
        // let side LiDAR readings bend this path.
        float correction = constrain(
            imu_correction,
            -max_forward_correction,
            max_forward_correction
        );

        float forward_pwm = getForwardApproachPWM(distance_mm);
        float left_pwm = constrain(forward_pwm + correction, 80, 180);
        float right_pwm = constrain(forward_pwm - correction, 80, 180);

        setDrivePWM(left_pwm, right_pwm);
    }

    bool finishCustomForwardIfNeeded(float distance_mm) {
        if (distance_mm < target_distance_mm) {
            return false;
        }

        completeCustomCommand();
        return true;
    }

    void completeCustomCommand() {
        stopMotors();
        command_index = custom_command_end_index;
        command_state = COMMAND_READY;

    }

    void updateForwardCommand() {
        float distance_mm = getAverageDistanceMM();
        char next_command =
            command_string[command_index + active_forward_command_count];

        // Front LiDAR collision check
        if (front_lidar_safety_enabled) {
            uint16_t front_distance_mm = front_lidar.readDistance();

            bool valid_front_reading = front_lidar.isReadingValid();

            if (valid_front_reading &&
                front_distance_mm <= front_stop_distance_mm) {

                Serial.print("Emergency stop: front wall at ");
                Serial.print(front_distance_mm);
                Serial.println(" mm");

                if (next_command =='l' || next_command == 'L' ||
                    next_command =='r' || next_command == 'R' ) {

                        Serial.print("Skipping forward and starting next turn");

                        completeCurrentCommand(active_forward_command_count);
                        return;
                    }

                Serial.print("No turn command available. Stopping.");
                finishTask();
                return;
            }

            // The LiDAR call blocks while the robot is still moving, so use a
            // fresh encoder value for the stopping decision.
            distance_mm = getAverageDistanceMM();
        }

        if (finishForwardRunIfNeeded(distance_mm, next_command)) {
            return;
        }

        float lidar_correction = getSideLidarAvoidanceCorrection();

        // Side LiDAR calls also block. Do not issue another forward command if
        // the encoder target was crossed while waiting for them.
        distance_mm = getAverageDistanceMM();
        if (finishForwardRunIfNeeded(distance_mm, next_command)) {
            return;
        }

        imu.update();
        float heading_error =
            IMU::wrapAngleDeg(imu.getYawDeg() - target_yaw_deg);

        float imu_correction = heading_pid.computeFromError(heading_error);

        float correction =
            imu_correction + lidar_correction;

        correction = constrain(
            correction,
            -max_forward_correction,
            max_forward_correction
        );

        float forward_pwm = getForwardApproachPWM(distance_mm);
        float left_pwm = constrain(forward_pwm + correction, 80, 180);
        float right_pwm = constrain(forward_pwm - correction, 80, 180);

        setDrivePWM(left_pwm, right_pwm);
    }

    float getSideLidarAvoidanceCorrection() {
        if (!side_lidar_avoidance_enabled) {
            resetSideLidarAvoidanceState();
            return 0;
        }

        uint16_t left_reading_mm = left_lidar.readDistance();
        bool left_wall = isValidSideWall(left_lidar, left_reading_mm);

        uint16_t right_reading_mm = right_lidar.readDistance();
        bool right_wall = isValidSideWall(right_lidar, right_reading_mm);

        float left_distance_mm = 0;
        float right_distance_mm = 0;

        if (left_wall) {
            left_distance_mm = filterSideDistance(
                left_reading_mm,
                filtered_left_distance_mm,
                left_lidar_filter_ready
            );
        } else {
            left_lidar_filter_ready = false;
        }

        if (right_wall) {
            right_distance_mm = filterSideDistance(
                right_reading_mm,
                filtered_right_distance_mm,
                right_lidar_filter_ready
            );
        } else {
            right_lidar_filter_ready = false;
        }

        // A side contributes only when it is inside the minimum-clearance
        // zone. At safe distances the side LiDARs do not influence steering.
        float left_intrusion_mm = left_wall
            ? max(0.0f, side_clearance_mm - left_distance_mm)
            : 0.0f;
        float right_intrusion_mm = right_wall
            ? max(0.0f, side_clearance_mm - right_distance_mm)
            : 0.0f;

        // Positive correction steers away from the left wall; negative
        // correction steers away from the right. If both sides are close,
        // steer toward whichever side has more clearance.
        float avoidance_error_mm = left_intrusion_mm - right_intrusion_mm;

        // Ignore very small intrusions caused by normal sensor noise.
        if (fabs(avoidance_error_mm) <= side_avoidance_deadband_mm) {
            avoidance_error_mm = 0;
        } else if (avoidance_error_mm > 0) {
            avoidance_error_mm -= side_avoidance_deadband_mm;
        } else {
            avoidance_error_mm += side_avoidance_deadband_mm;
        }

        float target_correction = constrain(
            side_avoidance_kp * avoidance_error_mm,
            -max_side_avoidance_correction,
            max_side_avoidance_correction
        );

        return slewSideLidarAvoidanceCorrection(target_correction);
    }

    bool isValidSideWall(Lidar& lidar, uint16_t distance_mm) {
        return lidar.isReadingValid() &&
               distance_mm >= min_side_wall_mm &&
               distance_mm <= max_side_wall_mm;
    }

    float filterSideDistance(uint16_t reading_mm,
                             float& filtered_distance_mm,
                             bool& filter_ready) {
        if (!filter_ready) {
            filtered_distance_mm = static_cast<float>(reading_mm);
            filter_ready = true;
        } else if (reading_mm < filtered_distance_mm) {
            // React immediately when clearance is shrinking. Only smooth the
            // release so avoidance is not delayed as the robot nears a wall.
            filtered_distance_mm = static_cast<float>(reading_mm);
        } else {
            filtered_distance_mm += side_lidar_filter_alpha *
                (static_cast<float>(reading_mm) - filtered_distance_mm);
        }

        return filtered_distance_mm;
    }

    float slewSideLidarAvoidanceCorrection(float target_correction) {
        float change = target_correction - current_side_avoidance_correction;
        change = constrain(
            change,
            -max_side_avoidance_step,
            max_side_avoidance_step
        );

        current_side_avoidance_correction += change;
        return current_side_avoidance_correction;
    }

    void resetSideLidarAvoidanceState() {
        left_lidar_filter_ready = false;
        right_lidar_filter_ready = false;
        filtered_left_distance_mm = 0;
        filtered_right_distance_mm = 0;
        current_side_avoidance_correction = 0;
    }

    void updateTurnCommand() {
        float yaw_error =
            IMU::wrapAngleDeg(target_yaw_deg - imu.getYawDeg());

        if (fabs(yaw_error) <= turn_tolerance_deg) {
            stopMotors();

            if (turn_settle_start_ms == 0) {
                turn_settle_start_ms = millis();
            }

            if (millis() - turn_settle_start_ms >= turn_settle_time_ms) {
                completeCurrentCommand();
            }

            return;
        }

        turn_settle_start_ms = 0;
        float pwm = getTurnPWM(yaw_error);

        setDrivePWM(-pwm, pwm);
    }

    float getTurnPWM(float yaw_error) {
        float pwm = turn_pid.computeFromError(yaw_error);
        float minimum_pwm =
            (fabs(yaw_error) <= turn_slow_angle_deg)
                ? min_turn_near_target_pwm
                : min_turn_pwm;

        if (yaw_error > 0) {
            return constrain(pwm, minimum_pwm, max_turn_pwm);
        }

        return constrain(pwm, -max_turn_pwm, -minimum_pwm);
    }

    void beginTurnController() {
        float initial_error =
            IMU::wrapAngleDeg(target_yaw_deg - imu.getYawDeg());

        turn_pid.reset(initial_error);
        turn_settle_start_ms = 0;
        resetSideLidarAvoidanceState();
    }

    bool finishForwardRunIfNeeded(float distance_mm, char next_command) {
        if (distance_mm < target_distance_mm) {
            return false;
        }

        if (next_command == '\0') {
            finishTask();
        } else {
            completeCurrentCommand(active_forward_command_count);
        }

        return true;
    }

    float getForwardApproachPWM(float distance_mm) {
        float remaining_mm = target_distance_mm - distance_mm;

        if (remaining_mm >= forward_slowdown_distance_mm) {
            return base_pwm;
        }

        float slowdown_ratio = constrain(
            remaining_mm / forward_slowdown_distance_mm,
            0.0f,
            1.0f
        );

        return min_forward_approach_pwm +
            (base_pwm - min_forward_approach_pwm) * slowdown_ratio;
    }

    void completeCurrentCommand(uint16_t command_count = 1) {
        stopMotors();

        command_index += command_count;

        char next_command = command_string[command_index];
        bool next_is_turn =
            next_command == 'l' || next_command == 'L' ||
            next_command == 'r' || next_command == 'R';

        command_pause_active =
            command_state == COMMAND_FORWARD && next_is_turn;
        command_pause_start_ms = millis();
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
    // A tighter deadband plus a longer settle time prevents a command from
    // completing while the chassis is still coasting through the target.
    float turn_tolerance_deg = 1.5f;
    unsigned long turn_settle_start_ms = 0;
    const unsigned long turn_settle_time_ms = 250;
    const float max_turn_pwm = 80.0f;
    const float min_turn_pwm = 50.0f;
    const float min_turn_near_target_pwm = 32.0f;
    const float turn_slow_angle_deg = 20.0f;
    const float max_forward_correction = 35.0;
    const float forward_slowdown_distance_mm = 90.0;
    const float min_forward_approach_pwm = 100.0;

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

    bool side_lidar_avoidance_enabled = false;
    float side_clearance_mm = 50.0;
    const float side_avoidance_kp = 0.67;
    const float max_side_avoidance_correction = 18.0;
    const float max_side_avoidance_step = 3.0;
    const float side_avoidance_deadband_mm = 3.0;
    const float side_lidar_filter_alpha = 0.35;
    const uint16_t min_side_wall_mm = 1;
    const uint16_t max_side_wall_mm = 100;
    float filtered_left_distance_mm = 0;
    float filtered_right_distance_mm = 0;
    float current_side_avoidance_correction = 0;
    bool left_lidar_filter_ready = false;
    bool right_lidar_filter_ready = false;
    bool front_lidar_safety_enabled = false;
    uint16_t front_stop_distance_mm = 40;

    const char* command_string = nullptr;
    uint16_t command_index = 0;
    uint16_t active_forward_command_count = 1;
    CommandState command_state = COMMAND_READY;
    float custom_angle_deg = 0.0f;
    float custom_distance_mm = 0.0f;
    uint16_t custom_command_end_index = 0;
    const float custom_angle_tolerance_deg = 0.1f;
    bool command_pause_active = false;
    unsigned long command_pause_start_ms = 0;
    const unsigned long command_pause_time_ms = 80;
    const float maze_cell_distance_mm = 180.0;

};

}  // namespace mtrn3100
