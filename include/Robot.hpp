#pragma once

#include <Arduino.h>
#include <math.h>

#include "Encoder.hpp"
#include "ExtendedKalmanFilter.hpp"
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
          ekf(wheel_base_mm),
          heading_pid(2.0, 0.0, 0.06),
          distance_pid(1.2, 0.02, 0.04),
          turn_pid(1.6, 0.0, 0.10) {}

    void begin() {
        left_encoder.begin();
        right_encoder.begin();

        beginSensors();

        imu.begin();
        imu.zeroYaw();
        initialiseStateEstimator();

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


    void startStraightLine(float distance_mm, int16_t pwm = 130) {
        task = TASK_STRAIGHT_LINE;
        finished = false;

        target_distance_mm = distance_mm;
        base_pwm = constrain(abs(pwm), 80, 180);

        start_left_rotation = left_encoder.getRotation();
        start_right_rotation = right_encoder.getRotation();

        target_yaw_deg = getControlHeadingDeg();
        heading_pid.reset();
        resetLidarCenteringState();

        Serial.print("Straight line target yaw: ");
        Serial.println(target_yaw_deg);
    }

    void startWallDistance(float front_distance_mm = 100.0f) {
        task = TASK_WALL_DISTANCE;
        finished = false;

        target_wall_distance_mm = front_distance_mm;
        target_yaw_deg = getControlHeadingDeg();

        distance_pid.reset();
        heading_pid.reset();

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
        target_yaw_deg = IMU::wrapAngleDeg(
            getControlHeadingDeg() + angle_deg
        );
        beginTurnController();
    }

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
        resetLidarCenteringState();
    }

    void enableLidarCentering(bool enabled, float target_side_distance_mm = 50.0) {
        lidar_centering_enabled = enabled;
        side_wall_target_mm = target_side_distance_mm;
        resetLidarCenteringState();
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
        target_yaw_deg = IMU::wrapAngleDeg(
            getControlHeadingDeg() + angle_deg
        );

        beginTurnController();

        Serial.print("Turn hold target yaw: ");
        Serial.println(target_yaw_deg);
    }

    void update() {
        updateStateEstimate();

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

    float getPoseXMM() const {
        return ekf.getXMM();
    }

    float getPoseYMM() const {
        return ekf.getYMM();
    }

    float getHeadingDeg() {
        return ekf.isInitialised()
            ? ekf.getHeadingDeg()
            : imu.getYawDeg();
    }

    void stop() {
        stopMotors();
        task = TASK_IDLE;
        finished = true;
    }

private:
    // Keep the motor-control loop tied to the direct IMU measurement. The EKF
    // heading also contains wheel-odometry prediction, so feeding it back into
    // wheel control can amplify encoder or wheel-radius mismatch. The EKF is
    // still updated continuously and remains the published pose estimate.
    float getControlHeadingDeg() {
        return imu.getYawDeg();
    }

    void updateStraightLine() {
        float distance_mm = getAverageDistanceMM();

        if (distance_mm >= target_distance_mm) {
            finishTask();
            return;
        }

        float current_yaw = getControlHeadingDeg();

        // Positive if robot has rotated away from starting heading
        float heading_error = IMU::wrapAngleDeg(current_yaw - target_yaw_deg);

        float imu_correction = heading_pid.computeFromError(heading_error);
        float encoder_correction = getEncoderBalanceCorrection();
        float lidar_correction = getLidarCenteringCorrection();

        // Speed up the wheel nearest a wall and slow the opposite wheel,
        // steering the robot away from that wall.
        float correction =
            imu_correction + encoder_correction + lidar_correction;

        correction = constrain(
            correction,
            -max_forward_correction,
            max_forward_correction
        );

        float left_pwm = constrain(base_pwm + correction, 80, 180);
        float right_pwm = constrain(base_pwm - correction, 80, 180);

        setDrivePWM(left_pwm, right_pwm);

        const unsigned long now_ms = millis();
        if (now_ms - last_straight_telemetry_ms >= 500UL) {
            last_straight_telemetry_ms = now_ms;
            Serial.print("Yaw: ");
            Serial.print(current_yaw);
            Serial.print(" Error: ");
            Serial.print(heading_error);
            Serial.print(" Enc_Corr: ");
            Serial.print(encoder_correction);
            Serial.print(" Distance: ");
            Serial.print(distance_mm);
            Serial.print(" L_PWM: ");
            Serial.print(left_pwm);
            Serial.print(" R_PWM: ");
            Serial.println(right_pwm);
        }
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
        float current_yaw = getControlHeadingDeg();

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
        float current_yaw = getControlHeadingDeg();

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
            target_yaw_deg = getControlHeadingDeg();

            heading_pid.reset();
            resetLidarCenteringState();
            command_state = COMMAND_FORWARD;
        }

        else if (command == 'l' || command == 'L') {
            target_yaw_deg =
                IMU::wrapAngleDeg(getControlHeadingDeg() + 90.0);

            beginTurnController();
            command_state = COMMAND_TURN;
        }

        else if (command == 'r' || command == 'R') {
            target_yaw_deg =
                IMU::wrapAngleDeg(getControlHeadingDeg() - 90.0);

            beginTurnController();
            command_state = COMMAND_TURN;
        }
        else {

            command_index++;

        }

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

        float lidar_correction = getLidarCenteringCorrection();

        // Side LiDAR calls also block. Do not issue another forward command if
        // the encoder target was crossed while waiting for them.
        distance_mm = getAverageDistanceMM();
        if (finishForwardRunIfNeeded(distance_mm, next_command)) {
            return;
        }

        updateStateEstimate();
        float heading_error =
            IMU::wrapAngleDeg(getControlHeadingDeg() - target_yaw_deg);

        float imu_correction = heading_pid.computeFromError(heading_error);
        float encoder_correction = getEncoderBalanceCorrection();

        float forward_pwm = getForwardApproachPWM(distance_mm);

        // Emergency side-wall correction gets priority over IMU/encoder
        // straight-line control and is allowed to slow the inside wheel
        // below the normal 80 PWM floor.
        if (fabs(lidar_correction) >= emergency_correction_threshold) {
            float left_pwm = constrain(
                forward_pwm + lidar_correction,
                0.0f,
                180.0f
            );

            float right_pwm = constrain(
                forward_pwm - lidar_correction,
                0.0f,
                180.0f
            );

            Serial.print("SIDE EMERGENCY DRIVE  L_PWM: ");
            Serial.print(left_pwm);
            Serial.print(" R_PWM: ");
            Serial.println(right_pwm);

            setDrivePWM(left_pwm, right_pwm);
            return;
        }

        float correction =
            imu_correction + encoder_correction + lidar_correction;

        correction = constrain(
            correction,
            -max_forward_correction,
            max_forward_correction
        );

        float left_pwm = constrain(forward_pwm + correction, 80, 180);
        float right_pwm = constrain(forward_pwm - correction, 80, 180);

        setDrivePWM(left_pwm, right_pwm);
    }

    float getLidarCenteringCorrection() {
        if (!lidar_centering_enabled) {
            resetLidarCenteringState();
            return 0;
        }

        uint16_t left_reading_mm = left_lidar.readDistance();
        bool left_wall = isValidSideWall(left_lidar, left_reading_mm);

        uint16_t right_reading_mm = right_lidar.readDistance();
        bool right_wall = isValidSideWall(right_lidar, right_reading_mm);

        // ============================================================
        // EMERGENCY SIDE WALL AVOIDANCE
        // Use RAW readings so filtering cannot delay the emergency.
        // ============================================================

        if (left_wall && right_wall &&
            left_reading_mm < emergency_side_distance_mm &&
            right_reading_mm < emergency_side_distance_mm) {

            if (left_reading_mm < right_reading_mm) {
                Serial.print("!!! LEFT EMERGENCY: ");
                Serial.println(left_reading_mm);

                current_lidar_correction = emergency_lidar_correction;
            } else {
                Serial.print("!!! RIGHT EMERGENCY: ");
                Serial.println(right_reading_mm);

                current_lidar_correction = -emergency_lidar_correction;
            }

            return current_lidar_correction;
        }

        if (left_wall && left_reading_mm < emergency_side_distance_mm) {
            Serial.print("!!! LEFT EMERGENCY: ");
            Serial.println(left_reading_mm);

            current_lidar_correction = emergency_lidar_correction;
            return current_lidar_correction;
        }

        if (right_wall && right_reading_mm < emergency_side_distance_mm) {
            Serial.print("!!! RIGHT EMERGENCY: ");
            Serial.println(right_reading_mm);

            current_lidar_correction = -emergency_lidar_correction;
            return current_lidar_correction;
        }

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

        uint8_t wall_state =
            (left_wall ? 0x01 : 0x00) |
            (right_wall ? 0x02 : 0x00);

        if (wall_state == 0) {
            resetLidarCenteringState();
            return 0;
        }

        // If visible walls change, reset the previous correction.
        if (wall_state != lidar_wall_state) {
            lidar_wall_state = wall_state;
            current_lidar_correction = 0;
        }

        // ============================================================
        // NORMAL GENTLE SIDE LIDAR CORRECTION
        // ============================================================

        float wall_error_mm = 0;

        if (left_wall && right_wall) {
            wall_error_mm = right_distance_mm - left_distance_mm;

        } else if (left_wall &&
                   left_distance_mm < side_wall_target_mm) {

            wall_error_mm =
                side_wall_target_mm - left_distance_mm;

        } else if (right_wall &&
                   right_distance_mm < side_wall_target_mm) {

            wall_error_mm =
                right_distance_mm - side_wall_target_mm;
        }

        // Ignore small differences caused by sensor noise.
        if (fabs(wall_error_mm) <= lidar_centering_deadband_mm) {
            wall_error_mm = 0;
        } else {
            if (wall_error_mm > 0) {
                wall_error_mm -= lidar_centering_deadband_mm;
            } else {
                wall_error_mm += lidar_centering_deadband_mm;
            }
        }

        float target_correction = constrain(
            lidar_centering_kp * wall_error_mm,
            -max_lidar_correction,
            max_lidar_correction
        );

        // Normal correction remains smooth.
        return slewLidarCorrection(target_correction);
    }

    bool isValidSideWall(Lidar& lidar, uint16_t distance_mm) {
        return lidar.isReadingValid() &&
            distance_mm >= min_side_wall_mm &&
            distance_mm <= max_side_wall_mm;
    }

    float filterSideDistance(
        uint16_t reading_mm,
        float& filtered_distance_mm,
        bool& filter_ready
    ) {
        if (!filter_ready) {
            filtered_distance_mm =
                static_cast<float>(reading_mm);
            filter_ready = true;
        } else {
            filtered_distance_mm +=
                side_lidar_filter_alpha *
                (static_cast<float>(reading_mm)
                - filtered_distance_mm);
        }

        return filtered_distance_mm;
    }

    float slewLidarCorrection(float target_correction) {
        float change =
            target_correction - current_lidar_correction;

        change = constrain(
            change,
            -max_lidar_correction_step,
            max_lidar_correction_step
        );

        current_lidar_correction += change;

        return current_lidar_correction;
    }
    
    void resetLidarCenteringState() {
        left_lidar_filter_ready = false;
        right_lidar_filter_ready = false;
        filtered_left_distance_mm = 0;
        filtered_right_distance_mm = 0;
        current_lidar_correction = 0;
        lidar_wall_state = 0;
    }

    void updateTurnCommand() {
        float yaw_error =
            IMU::wrapAngleDeg(target_yaw_deg - getControlHeadingDeg());

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
            IMU::wrapAngleDeg(target_yaw_deg - getControlHeadingDeg());

        turn_pid.reset(initial_error);
        turn_settle_start_ms = 0;
        resetLidarCenteringState();
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

    void completeCurrentCommand(uint8_t command_count = 1) {
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

    float getEncoderBalanceCorrection() {
        const float left_distance_mm = fabs(
            left_encoder.getRotation() - start_left_rotation
        ) * wheel_radius_mm;
        const float right_distance_mm = fabs(
            right_encoder.getRotation() - start_right_rotation
        ) * wheel_radius_mm;

        // Positive correction speeds up the left wheel and slows the right.
        // Therefore, if the left wheel has travelled farther, the correction
        // is negative and gently lets the right wheel catch up.
        return constrain(
            encoder_balance_kp * (right_distance_mm - left_distance_mm),
            -max_encoder_balance_correction,
            max_encoder_balance_correction
        );
    }

    void initialiseStateEstimator() {
        previous_left_estimator_rotation = left_encoder.getRotation();
        previous_right_estimator_rotation = right_encoder.getRotation();
        left_encoder_polarity = 0;
        right_encoder_polarity = 0;
        last_left_motion_direction = 0;
        last_right_motion_direction = 0;

        ekf.reset(0.0f, 0.0f, imu.getYawDeg());
        ekf.begin(imu.getYawDeg());
    }

    void updateStateEstimate() {
        imu.update();

        float left_rotation = left_encoder.getRotation();
        float right_rotation = right_encoder.getRotation();
        float raw_left_delta =
            left_rotation - previous_left_estimator_rotation;
        float raw_right_delta =
            right_rotation - previous_right_estimator_rotation;

        previous_left_estimator_rotation = left_rotation;
        previous_right_estimator_rotation = right_rotation;

        float left_delta = normaliseEncoderDelta(
            raw_left_delta,
            last_left_motion_direction,
            left_encoder_polarity
        );
        float right_delta = normaliseEncoderDelta(
            raw_right_delta,
            last_right_motion_direction,
            right_encoder_polarity
        );

        ekf.updateWheelDistances(
            left_delta * wheel_radius_mm,
            right_delta * wheel_radius_mm,
            imu.getYawDeg()
        );
    }

    float normaliseEncoderDelta(float raw_delta,
                                int8_t commanded_direction,
                                int8_t& encoder_polarity) {
        if (fabs(raw_delta) < 0.000001f) {
            return 0.0f;
        }

        if (encoder_polarity == 0 && commanded_direction != 0) {
            int8_t raw_direction = raw_delta > 0.0f ? 1 : -1;
            encoder_polarity = commanded_direction * raw_direction;
        }

        // Until this wheel has moved under a known command, ignoring its
        // delta is safer than allowing an unknown sign to rotate the EKF.
        if (encoder_polarity == 0) {
            return 0.0f;
        }

        return raw_delta * encoder_polarity;
    }

    void setDrivePWM(float left_pwm, float right_pwm) {
        if (left_pwm > 0.0f) {
            last_left_motion_direction = 1;
        } else if (left_pwm < 0.0f) {
            last_left_motion_direction = -1;
        }

        if (right_pwm > 0.0f) {
            last_right_motion_direction = 1;
        } else if (right_pwm < 0.0f) {
            last_right_motion_direction = -1;
        }

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

    ExtendedKalmanFilter ekf;

    float previous_left_estimator_rotation = 0.0f;
    float previous_right_estimator_rotation = 0.0f;
    int8_t left_encoder_polarity = 0;
    int8_t right_encoder_polarity = 0;
    int8_t last_left_motion_direction = 0;
    int8_t last_right_motion_direction = 0;

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
    float turn_tolerance_deg = 3;
    unsigned long turn_settle_start_ms = 0;
    const unsigned long turn_settle_time_ms = 100;
    const float max_turn_pwm = 90.0;
    const float min_turn_pwm = 55.0;
    const float min_turn_near_target_pwm = 45.0;
    const float turn_slow_angle_deg = 15.0;
    const float max_forward_correction = 35.0;
    const float encoder_balance_kp = 0.3f;
    const float max_encoder_balance_correction = 12.0f;
    const float forward_slowdown_distance_mm = 90.0;
    const float min_forward_approach_pwm = 100.0;
    unsigned long last_straight_telemetry_ms = 0;

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
    float side_wall_target_mm = 60.0;
    const float lidar_centering_kp = 0.75f;
    const float max_lidar_correction = 25.0f;
    const float max_lidar_correction_step = 4.0f;
    const float lidar_centering_deadband_mm = 3.0;
    const float side_lidar_filter_alpha = 0.35;

    // Emergency side-wall avoidance.
    // Raw LiDAR reading below 45 mm immediately overrides normal
    // IMU/encoder correction with a strong steering command.
    const uint16_t emergency_side_distance_mm = 45;
    const float emergency_lidar_correction = 45.0f;
    const float emergency_correction_threshold = 44.0f;

    const uint16_t min_side_wall_mm = 1;
    const uint16_t max_side_wall_mm = 100;
    float filtered_left_distance_mm = 0;
    float filtered_right_distance_mm = 0;
    float current_lidar_correction = 0;
    uint8_t lidar_wall_state = 0;
    bool left_lidar_filter_ready = false;
    bool right_lidar_filter_ready = false;
    bool front_lidar_safety_enabled = false;
    uint16_t front_stop_distance_mm = 40;

    const char* command_string = nullptr;
    uint8_t command_index = 0;
    uint8_t active_forward_command_count = 1;
    CommandState command_state = COMMAND_READY;
    bool command_pause_active = false;
    unsigned long command_pause_start_ms = 0;
    const unsigned long command_pause_time_ms = 80;
    const float maze_cell_distance_mm = 180.0;

};

}  // namespace mtrn3100
