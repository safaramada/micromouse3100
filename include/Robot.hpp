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
    TASK_TURN
};

// Hardware and motion control used by Task 4.3 autonomous mapping.
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
          float wheel_radius_mm = 16.0)
        : left_motor(left_motor),
          right_motor(right_motor),
          left_encoder(left_encoder),
          right_encoder(right_encoder),
          front_lidar(front_lidar),
          left_lidar(left_lidar),
          right_lidar(right_lidar),
          imu(imu),
          wheel_radius_mm(wheel_radius_mm),
          heading_pid(2.0, 0.0, 0.06),
          turn_pid(1.6, 0.0, 0.10) {}

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


    void startStraightLine(float distance_mm,
                           int16_t pwm = 130,
                           uint16_t front_stop_distance_mm = 0) {
        task = TASK_STRAIGHT_LINE;
        finished = false;
        front_wall_stopped = false;

        target_distance_mm = distance_mm;
        base_pwm = constrain(abs(pwm), 80, 180);
        this->front_stop_distance_mm = front_stop_distance_mm;

        start_left_rotation = left_encoder.getRotation();
        start_right_rotation = right_encoder.getRotation();

        target_yaw_deg = imu.getYawDeg();
        heading_pid.reset();
        resetSideLidarAvoidanceState();

    }

    void startTurn(float angle_deg) {
        task = TASK_TURN;
        finished = false;
        front_wall_stopped = false;
        target_yaw_deg = IMU::wrapAngleDeg(imu.getYawDeg() + angle_deg);
        beginTurnController();
    }

    void enableSideLidarAvoidance(bool enabled,
                                  float minimum_clearance_mm = 50.0f) {
        side_lidar_avoidance_enabled = enabled;
        side_clearance_mm = minimum_clearance_mm;
        resetSideLidarAvoidanceState();
    }

    void update() {
        imu.update();

        switch (task) {
            case TASK_STRAIGHT_LINE:
                updateStraightLine();
                break;

            case TASK_TURN:
                updateTurn();
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

    bool stoppedForFrontWall() const {
        return front_wall_stopped;
    }

    float getTravelledDistanceMM() {
        return getAverageDistanceMM();
    }

    bool senseFrontWall(uint16_t threshold_mm, bool& wall_present) {
        return senseWall(front_lidar, threshold_mm, wall_present);
    }

    bool senseLeftWall(uint16_t threshold_mm, bool& wall_present) {
        return senseWall(left_lidar, threshold_mm, wall_present);
    }

    bool senseRightWall(uint16_t threshold_mm, bool& wall_present) {
        return senseWall(right_lidar, threshold_mm, wall_present);
    }

    void stop() {
        stopMotors();
        task = TASK_IDLE;
        finished = true;
        front_wall_stopped = false;
    }

private:
    bool senseWall(Lidar& lidar,
                   uint16_t threshold_mm,
                   bool& wall_present) {
        const uint16_t distance_mm = lidar.readDistance();

        if (lidar.isReadingValid()) {
            wall_present = distance_mm <= threshold_mm;
            return true;
        }

        if (!lidar.isReady() || lidar.timedOut()) {
            return false;
        }

        // The original Task 1 driver rejects every non-zero range status.
        // For mapping, several of those statuses specifically mean that no
        // target was found within range, which means no adjacent wall.
        switch (lidar.getRangeStatus()) {
            case 6:   // early convergence: no target
            case 7:   // maximum convergence: no target
            case 8:   // no-target ignore threshold
            case 13:  // raw range overflow: target too far away
            case 15:  // range overflow: target too far away
                wall_present = false;
                return true;

            case 12:  // raw underflow: target extremely close
            case 14:  // underflow: target extremely close
                wall_present = true;
                return true;

            default:
                return false;
        }
    }

    void updateStraightLine() {
        float distance_mm = getAverageDistanceMM();

        if (distance_mm >= target_distance_mm) {
            finishTask();
            return;
        }

        // This is a final collision guard. AutonomousMapping performs a
        // separate, longer-range check before starting each cell movement.
        if (front_stop_distance_mm > 0) {
            bool front_wall = false;
            if (senseFrontWall(front_stop_distance_mm, front_wall) &&
                front_wall) {
                front_wall_stopped = true;
                finishTask();
                return;
            }

            // A single VL6180X reading can block for long enough to cross the
            // encoder target, so check the distance again before driving.
            distance_mm = getAverageDistanceMM();
            if (distance_mm >= target_distance_mm) {
                finishTask();
                return;
            }
        }

        float current_yaw = imu.getYawDeg();

        // Positive if robot has rotated away from starting heading
        float heading_error = IMU::wrapAngleDeg(current_yaw - target_yaw_deg);

        float imu_correction = heading_pid.computeFromError(heading_error);
        float lidar_correction = getSideLidarAvoidanceCorrection();

        // The two side readings also take time. Never send another forward
        // command if the cell target was crossed while waiting for them.
        distance_mm = getAverageDistanceMM();
        if (distance_mm >= target_distance_mm) {
            finishTask();
            return;
        }

        // If a side wall is too close, speed up the wheel nearest that wall
        // and slow the opposite wheel to steer away from it.
        float correction = imu_correction + lidar_correction;

        correction = constrain(
            correction,
            -max_forward_correction,
            max_forward_correction
        );

        const float forward_pwm = getForwardApproachPWM(distance_mm);
        float left_pwm = constrain(forward_pwm + correction, 80, 180);
        float right_pwm = constrain(forward_pwm - correction, 80, 180);

        setDrivePWM(left_pwm, right_pwm);

    }

    void updateTurn() {
        float current_yaw = imu.getYawDeg();

        float yaw_error = IMU::wrapAngleDeg(target_yaw_deg - current_yaw);

        // Stop after the heading remains within tolerance for the settle time.
        if (fabs(yaw_error) <= turn_tolerance_deg) {
            stopMotors();

            if (turn_settle_start_ms == 0) {
                turn_settle_start_ms = millis();
            }

            if (millis() - turn_settle_start_ms < turn_settle_time_ms) {
                finished = false;
                return;
            }

            finishTask();
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

    float getForwardApproachPWM(float distance_mm) const {
        const float remaining_mm = target_distance_mm - distance_mm;
        if (remaining_mm >= forward_slowdown_distance_mm) {
            return base_pwm;
        }

        const float slowdown_ratio = constrain(
            remaining_mm / forward_slowdown_distance_mm,
            0.0f,
            1.0f
        );
        return min_forward_approach_pwm +
            (base_pwm - min_forward_approach_pwm) * slowdown_ratio;
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
        stopMotors();
        task = TASK_IDLE;
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

    PIDController heading_pid;
    PIDController turn_pid;

    RobotTask task = TASK_IDLE;
    bool finished = true;
    bool front_wall_stopped = false;

    int16_t base_pwm = 120;
    uint16_t front_stop_distance_mm = 0;
    float target_distance_mm = 0;
    float target_yaw_deg = 0;
    float start_left_rotation = 0;
    float start_right_rotation = 0;
    float turn_tolerance_deg = 3;
    unsigned long turn_settle_start_ms = 0;
    const unsigned long turn_settle_time_ms = 100;
    const float max_turn_pwm = 90.0;
    const float min_turn_pwm = 55.0;
    const float min_turn_near_target_pwm = 45.0;
    const float turn_slow_angle_deg = 15.0;
    const float max_forward_correction = 35.0;
    const float forward_slowdown_distance_mm = 90.0;
    const float min_forward_approach_pwm = 100.0;

    bool side_lidar_avoidance_enabled = false;
    float side_clearance_mm = 50.0;
    const float side_avoidance_kp = 0.67;
    const float max_side_avoidance_correction = 18.0;
    const float max_side_avoidance_step = 6.0;
    const float side_avoidance_deadband_mm = 3.0;
    const float side_lidar_filter_alpha = 0.35;
    const uint16_t min_side_wall_mm = 1;
    const uint16_t max_side_wall_mm = 100;
    float filtered_left_distance_mm = 0;
    float filtered_right_distance_mm = 0;
    float current_side_avoidance_correction = 0;
    bool left_lidar_filter_ready = false;
    bool right_lidar_filter_ready = false;

};

}  // namespace mtrn3100
