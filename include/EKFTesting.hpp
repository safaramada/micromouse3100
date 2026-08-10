#pragma once

#include <Arduino.h>
#include <math.h>

#include "IMU.hpp"
#include "Robot.hpp"

namespace mtrn3100 {

// To test select one of the value in main.cpp (line 57) to true. Only one should be true.
// When all are false, the normal maze command program runs.

class EKFTesting {
public:
    explicit EKFTesting(Robot& robot) : robot(robot) {}

    void startStationaryDriftTest(unsigned long duration_ms = 10000UL) {
        beginTest(TEST_STATIONARY);
        test_duration_ms = duration_ms;
        robot.enableLidarCentering(false);
        robot.enableFrontLidarSafety(false);
        robot.stop();
        Serial.println(F("EKF_TEST:STATIONARY_DRIFT,STATUS:STARTED"));
    }

    void startStraightGapTest(float distance_mm = 1000.0f,
                              int16_t pwm = 110) {
        beginTest(TEST_STRAIGHT_GAP);
        // Isolate the EKF/IMU heading test from side-wall corrections.
        robot.enableLidarCentering(false);
        robot.enableFrontLidarSafety(false);
        robot.startStraightLine(distance_mm, pwm);
        Serial.print(F("EKF_TEST:STRAIGHT_GAP,STATUS:STARTED,DISTANCE_MM:"));
        Serial.println(distance_mm);
    }

    void startTurnReturnTest(float angle_deg = 90.0f) {
        beginTest(TEST_TURN_RETURN);
        turn_angle_deg = angle_deg;
        phase = 0;
        robot.enableLidarCentering(false);
        robot.enableFrontLidarSafety(false);
        robot.startTurn(turn_angle_deg);
        Serial.print(F("EKF_TEST:TURN_RETURN,STATUS:STARTED,ANGLE_DEG:"));
        Serial.println(turn_angle_deg);
    }

    void update() {
        if (!active) {
            return;
        }

        if (test == TEST_STATIONARY) {
            if (millis() - test_start_ms >= test_duration_ms) {
                finishTest(F("STATIONARY_DRIFT"));
            }
            return;
        }

        if (!robot.isFinished()) {
            return;
        }

        if (test == TEST_STRAIGHT_GAP) {
            finishTest(F("STRAIGHT_GAP"));
            return;
        }

        if (test == TEST_TURN_RETURN) {
            if (phase == 0) {
                phase = 1;
                delay(250);
                robot.startTurn(-turn_angle_deg);
                Serial.println(F("EKF_TEST:TURN_RETURN,STATUS:RETURNING"));
            } else {
                finishTest(F("TURN_RETURN"));
            }
        }
    }

    bool isActive() const { return active; }

private:
    enum TestType {
        TEST_NONE,
        TEST_STATIONARY,
        TEST_STRAIGHT_GAP,
        TEST_TURN_RETURN
    };

    void beginTest(TestType selected_test) {
        test = selected_test;
        active = true;
        test_start_ms = millis();
        initial_x_mm = robot.getPoseXMM();
        initial_y_mm = robot.getPoseYMM();
        initial_heading_deg = robot.getHeadingDeg();
    }

    void finishTest(const __FlashStringHelper* name) {
        robot.stop();
        active = false;

        const float delta_x_mm = robot.getPoseXMM() - initial_x_mm;
        const float delta_y_mm = robot.getPoseYMM() - initial_y_mm;
        const float displacement_mm =
            sqrt(delta_x_mm * delta_x_mm + delta_y_mm * delta_y_mm);
        const float heading_error_deg = IMU::wrapAngleDeg(
            robot.getHeadingDeg() - initial_heading_deg
        );

        Serial.print(F("EKF_TEST:"));
        Serial.print(name);
        Serial.print(F(",STATUS:FINISHED,DELTA_X_MM:"));
        Serial.print(delta_x_mm);
        Serial.print(F(",DELTA_Y_MM:"));
        Serial.print(delta_y_mm);
        Serial.print(F(",DISPLACEMENT_MM:"));
        Serial.print(displacement_mm);
        Serial.print(F(",HEADING_ERROR_DEG:"));
        Serial.println(heading_error_deg);
    }

    Robot& robot;
    TestType test = TEST_NONE;
    bool active = false;
    uint8_t phase = 0;
    unsigned long test_start_ms = 0;
    unsigned long test_duration_ms = 0;
    float turn_angle_deg = 90.0f;
    float initial_x_mm = 0.0f;
    float initial_y_mm = 0.0f;
    float initial_heading_deg = 0.0f;
};

}  // namespace mtrn3100
