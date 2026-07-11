#pragma once

#include <Arduino.h>
#include "Robot.hpp"

class StraightLineTracking {
public:
    StraightLineTracking(Robot& robot) : robot(robot) {}

    void begin() {
        robot.encoders.reset();

        delay(1000); // keep still while IMU settles

        targetYaw = robot.imu.getYaw();

        Serial.print("Target yaw: ");
        Serial.println(targetYaw);

        finished = false;
    }

    void update() {
        if (finished) {
            robot.stopMotors();
            return;
        }

        float currentYaw = robot.imu.getYaw();
        float yawError = angleError(currentYaw, targetYaw);

        float correction = Kp * yawError;

        int leftPWM  = baseLeftPWM  - correction;
        int rightPWM = baseRightPWM - correction;

        leftPWM = constrain(leftPWM, 80, 180);
        rightPWM = constrain(rightPWM, -180, -80);

        robot.leftMotor.setPWM(leftPWM);
        robot.rightMotor.setPWM(rightPWM);

        float distanceMM = robot.encoders.getAverageDistanceMM();

        Serial.print("Yaw: ");
        Serial.print(currentYaw);
        Serial.print(" Error: ");
        Serial.print(yawError);
        Serial.print(" Distance: ");
        Serial.println(distanceMM);

        if (distanceMM >= 1000) {
            robot.stopMotors();
            finished = true;
        }
    }

private:
    Robot& robot;

    float targetYaw = 0;
    bool finished = false;

    float Kp = 4.0;

    int baseLeftPWM = 155;
    int baseRightPWM = -150;

    float angleError(float current, float target) {
        float error = current - target;

        while (error > 180) {
            error -= 360;
        }

        while (error < -180) {
            error += 360;
        }

        return error;
    }
};