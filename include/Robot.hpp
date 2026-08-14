#pragma once

#include <Arduino.h>
#include <math.h>

#include "Encoder.hpp"
#include "IMU.hpp"
#include "Lidar.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"

#ifndef LIDAR_DIAGNOSTICS
#define LIDAR_DIAGNOSTICS 0
#endif

namespace mtrn3100 {


enum RobotTask {
    TASK_IDLE,
    TASK_STRAIGHT_LINE,
    TASK_TURN
};

enum EmergencyStopReason : uint8_t {
    EMERGENCY_NONE,
    EMERGENCY_FRONT
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
        emergency_stop_reason = EMERGENCY_NONE;

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
        emergency_stop_reason = EMERGENCY_NONE;
        target_yaw_deg = IMU::wrapAngleDeg(imu.getYawDeg() + angle_deg);
        beginTurnController();
    }

    void enableSideLidarAvoidance(bool enabled,
                                  float minimum_clearance_mm = 50.0f) {
        side_lidar_avoidance_enabled = enabled;
        side_clearance_mm = minimum_clearance_mm;
        resetSideLidarAvoidanceState();
    }

    void enableEmergencyProtection(bool enabled,
                                   uint16_t front_distance_mm = 45,
                                   uint16_t side_distance_mm = 25) {
        emergency_protection_enabled = enabled;
        emergency_front_distance_mm = front_distance_mm;
        emergency_side_distance_mm = side_distance_mm;
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

    EmergencyStopReason getEmergencyStopReason() const {
        return emergency_stop_reason;
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
        emergency_stop_reason = EMERGENCY_NONE;
    }

private:
    struct SideLidarState {
        uint16_t samples[3] = {0, 0, 0};
        uint8_t sample_count = 0;
        uint8_t next_sample = 0;
        uint8_t invalid_streak = 0;
        uint8_t close_confirmation_count = 0;
        float filtered_distance_mm = 0;
        bool filter_ready = false;
        bool avoidance_active = false;
    };

    bool senseWall(Lidar& lidar,
                   uint16_t threshold_mm,
                   bool& wall_present) {
        const uint16_t distance_mm = lidar.readDistance();

        switch (lidar.getReadingResult()) {
            case Lidar::READING_VALID:
                wall_present = distance_mm <= threshold_mm;
                return true;

            case Lidar::READING_NO_TARGET:
                wall_present = false;
                return true;

            case Lidar::READING_TOO_CLOSE:
                wall_present = true;
                return true;

            case Lidar::READING_INVALID:
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

        // Close-range protection is independent of mapping. It remains active
        // throughout forward motion even after the stationary map check.
        const uint16_t active_front_stop_mm = front_stop_distance_mm > 0
            ? front_stop_distance_mm
            : emergency_front_distance_mm;
        if ((emergency_protection_enabled || front_stop_distance_mm > 0) &&
            active_front_stop_mm > 0) {
            bool front_wall = false;
            if (senseFrontWall(active_front_stop_mm, front_wall) &&
                front_wall) {
                front_wall_stopped = true;
                emergency_stop_reason = EMERGENCY_FRONT;
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

        if (emergency_stop_reason != EMERGENCY_NONE) {
            finishTask();
            return;
        }

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
        if (!side_lidar_avoidance_enabled &&
            !emergency_protection_enabled) {
            resetSideLidarAvoidanceState();
            return 0;
        }

        uint16_t left_reading_mm = left_lidar.readDistance();
        uint16_t right_reading_mm = right_lidar.readDistance();

        if (emergency_protection_enabled) {
            const bool left_emergency_close = isEmergencyClose(
                left_lidar,
                left_reading_mm,
                emergency_side_distance_mm
            );
            const bool right_emergency_close = isEmergencyClose(
                right_lidar,
                right_reading_mm,
                emergency_side_distance_mm
            );

            // Side walls never end the 180 mm cell action and never command a
            // reverse. A close wall instead produces the strongest permitted
            // steering away while the forward encoder target remains active.
            if (left_emergency_close && !right_emergency_close) {
                current_side_avoidance_correction =
                    max_side_avoidance_correction;
                return current_side_avoidance_correction;
            }
            if (right_emergency_close && !left_emergency_close) {
                current_side_avoidance_correction =
                    -max_side_avoidance_correction;
                return current_side_avoidance_correction;
            }
            if (left_emergency_close && right_emergency_close) {
                // In a genuinely narrow corridor, steer away from the nearer
                // side when both numeric readings are valid. Equal/underflow
                // readings continue straight rather than oscillating.
                if (left_lidar.isReadingValid() &&
                    right_lidar.isReadingValid()) {
                    if (left_reading_mm < right_reading_mm) {
                        current_side_avoidance_correction =
                            max_side_avoidance_correction;
                    } else if (right_reading_mm < left_reading_mm) {
                        current_side_avoidance_correction =
                            -max_side_avoidance_correction;
                    } else {
                        current_side_avoidance_correction = 0;
                    }
                    return current_side_avoidance_correction;
                }

                current_side_avoidance_correction = 0;
                return 0;
            }
        }

        if (!side_lidar_avoidance_enabled) {
            resetSideLidarAvoidanceState();
            return 0;
        }

        float left_distance_mm = 0;
        float right_distance_mm = 0;
        bool left_fresh_sample = false;
        bool right_fresh_sample = false;

        const bool left_distance_available = updateSideDistance(
            left_lidar,
            left_reading_mm,
            left_side_state,
            left_distance_mm,
            left_fresh_sample
        );
        const bool right_distance_available = updateSideDistance(
            right_lidar,
            right_reading_mm,
            right_side_state,
            right_distance_mm,
            right_fresh_sample
        );

        // Two close samples engage avoidance. It then stays engaged until the
        // wall clears a wider release threshold, preventing threshold chatter.
        const float left_intrusion_mm = getSideIntrusion(
            left_distance_available,
            left_fresh_sample,
            left_distance_mm,
            left_side_state
        );
        const float right_intrusion_mm = getSideIntrusion(
            right_distance_available,
            right_fresh_sample,
            right_distance_mm,
            right_side_state
        );

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

        const float correction =
            slewSideLidarAvoidanceCorrection(target_correction);

#if LIDAR_DIAGNOSTICS
        logLidarDiagnostics(
            left_reading_mm,
            right_reading_mm,
            left_distance_mm,
            right_distance_mm,
            correction
        );
#endif

        return correction;
    }

    bool isEmergencyClose(Lidar& lidar,
                          uint16_t distance_mm,
                          uint16_t threshold_mm) {
        const Lidar::ReadingResult result = lidar.getReadingResult();
        if (result == Lidar::READING_VALID) {
            return distance_mm > 0 && distance_mm <= threshold_mm;
        }

        return result == Lidar::READING_TOO_CLOSE;
    }

    static uint16_t medianOfThree(uint16_t a,
                                  uint16_t b,
                                  uint16_t c) {
        return a + b + c - min(a, min(b, c)) - max(a, max(b, c));
    }

    void clearSideLidarState(SideLidarState& state) {
        state = SideLidarState();
    }

    bool updateSideDistance(Lidar& lidar,
                            uint16_t reading_mm,
                            SideLidarState& state,
                            float& filtered_distance_mm,
                            bool& fresh_sample) {
        fresh_sample = false;
        const Lidar::ReadingResult result = lidar.getReadingResult();

        if (result == Lidar::READING_TOO_CLOSE) {
            // Emergency protection handles this immediately when enabled. The
            // zero-distance fallback also keeps avoidance safe when it is not.
            state.invalid_streak = 0;
            state.filtered_distance_mm = 0;
            state.filter_ready = true;
            filtered_distance_mm = 0;
            fresh_sample = true;
            return true;
        }

        if (result == Lidar::READING_VALID &&
            reading_mm >= min_side_wall_mm &&
            reading_mm <= max_side_wall_mm) {
            state.invalid_streak = 0;
            fresh_sample = true;

            state.samples[state.next_sample] = reading_mm;
            state.next_sample = (state.next_sample + 1U) % 3U;
            if (state.sample_count < 3U) {
                state.sample_count++;
            }

            uint16_t median_mm = reading_mm;
            if (state.sample_count == 2U) {
                median_mm = static_cast<uint16_t>(
                    (state.samples[0] + state.samples[1]) / 2U
                );
            } else if (state.sample_count == 3U) {
                median_mm = medianOfThree(
                    state.samples[0],
                    state.samples[1],
                    state.samples[2]
                );
            }

            if (!state.filter_ready ||
                median_mm < state.filtered_distance_mm) {
                // The median rejects a lone low outlier, so a confirmed
                // reduction in clearance can still be acted on immediately.
                state.filtered_distance_mm = static_cast<float>(median_mm);
                state.filter_ready = true;
            } else {
                state.filtered_distance_mm += side_lidar_filter_alpha *
                    (static_cast<float>(median_mm) -
                     state.filtered_distance_mm);
            }

            filtered_distance_mm = state.filtered_distance_mm;
            return true;
        }

        if (result == Lidar::READING_INVALID && state.filter_ready &&
            state.invalid_streak < max_held_invalid_samples) {
            // Hold one prior result so a single I2C/ranging glitch cannot
            // reset avoidance. It does not count as a confirmation sample.
            state.invalid_streak++;
            if (!state.avoidance_active) {
                state.close_confirmation_count = 0;
            }
            filtered_distance_mm = state.filtered_distance_mm;
            return true;
        }

        // A confirmed open/far result, or repeated invalid samples, releases
        // this side rather than steering from stale data indefinitely.
        clearSideLidarState(state);
        return false;
    }

    float getSideIntrusion(bool distance_available,
                           bool fresh_sample,
                           float distance_mm,
                           SideLidarState& state) {
        if (!distance_available) {
            return 0;
        }

        const float release_distance_mm =
            side_clearance_mm + side_avoidance_hysteresis_mm;

        if (state.avoidance_active) {
            if (fresh_sample && distance_mm >= release_distance_mm) {
                state.avoidance_active = false;
                state.close_confirmation_count = 0;
            }
        } else if (fresh_sample) {
            if (distance_mm < side_clearance_mm) {
                if (state.close_confirmation_count <
                    side_close_confirmation_samples) {
                    state.close_confirmation_count++;
                }
                if (state.close_confirmation_count >=
                    side_close_confirmation_samples) {
                    state.avoidance_active = true;
                }
            } else {
                state.close_confirmation_count = 0;
            }
        }

        if (!state.avoidance_active) {
            return 0;
        }

        return max(0.0f, release_distance_mm - distance_mm);
    }

#if LIDAR_DIAGNOSTICS
    void logLidarDiagnostics(uint16_t left_raw_mm,
                             uint16_t right_raw_mm,
                             float left_filtered_mm,
                             float right_filtered_mm,
                             float correction) {
        const unsigned long now = millis();
        if (now - last_lidar_diagnostic_ms < 100) {
            return;
        }
        last_lidar_diagnostic_ms = now;

        Serial.print(F("L raw/status/filter="));
        Serial.print(left_raw_mm);
        Serial.print('/');
        Serial.print(left_lidar.getRangeStatus());
        Serial.print('/');
        Serial.print(left_filtered_mm, 1);
        Serial.print(F(" R="));
        Serial.print(right_raw_mm);
        Serial.print('/');
        Serial.print(right_lidar.getRangeStatus());
        Serial.print('/');
        Serial.print(right_filtered_mm, 1);
        Serial.print(F(" correction="));
        Serial.println(correction, 1);
    }
#endif

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
        clearSideLidarState(left_side_state);
        clearSideLidarState(right_side_state);
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
    EmergencyStopReason emergency_stop_reason = EMERGENCY_NONE;
    bool emergency_protection_enabled = false;

    int16_t base_pwm = 120;
    uint16_t front_stop_distance_mm = 0;
    uint16_t emergency_front_distance_mm = 45;
    uint16_t emergency_side_distance_mm = 25;
    float target_distance_mm = 0;
    float target_yaw_deg = 0;
    float start_left_rotation = 0;
    float start_right_rotation = 0;
    float turn_tolerance_deg = 3;
    unsigned long turn_settle_start_ms = 0;
    const unsigned long turn_settle_time_ms = 100;
    const float max_turn_pwm = 82.0;
    const float min_turn_pwm = 52.0;
    const float min_turn_near_target_pwm = 42.0;
    const float turn_slow_angle_deg = 15.0;
    const float max_forward_correction = 32.0;
    const float forward_slowdown_distance_mm = 90.0;
    const float min_forward_approach_pwm = 95.0;

    bool side_lidar_avoidance_enabled = false;
    float side_clearance_mm = 50.0;
    const float side_avoidance_kp = 0.62;
    const float max_side_avoidance_correction = 16.0;
    const float max_side_avoidance_step = 5.0;
    const float side_avoidance_deadband_mm = 3.0;
    const float side_avoidance_hysteresis_mm = 5.0;
    const float side_lidar_filter_alpha = 0.35;
    const uint8_t side_close_confirmation_samples = 2;
    const uint8_t max_held_invalid_samples = 1;
    const uint16_t min_side_wall_mm = 1;
    const uint16_t max_side_wall_mm = 100;
    SideLidarState left_side_state;
    SideLidarState right_side_state;
    float current_side_avoidance_correction = 0;
#if LIDAR_DIAGNOSTICS
    unsigned long last_lidar_diagnostic_ms = 0;
#endif

};

}  // namespace mtrn3100
