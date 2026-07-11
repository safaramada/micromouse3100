#include <Arduino.h>

#include "Motor.hpp"
#include "Encoder.hpp"
#include "IMU.hpp"
#include "Lidar.hpp"
#include "Robot.hpp"
#include "StraightLineTracking.hpp"

using namespace mtrn3100;

// Replace these pins with your real wiring
Motor leftMotor(11, 12);
Motor rightMotor(9, 10);

// Encoder channel A pins should be interrupt-capable pins
Encoder leftEncoder(2, 4, 700, false);
Encoder rightEncoder(3, 5, 700, false);

// Lidar is not used for straight-line, but Robot requires these objects
Lidar frontLidar;
Lidar leftLidar;
Lidar rightLidar;

IMU imu;

Robot robot(
    leftMotor,
    rightMotor,
    leftEncoder,
    rightEncoder,
    frontLidar,
    leftLidar,
    rightLidar,
    imu,
    16.0,  // wheel radius in mm
    80.0   // wheel base in mm
);

StraightLineTracking straightLineTask(robot);

void setup() {
    Serial.begin(115200);
    delay(1000);

    robot.begin();

    delay(1000);

    straightLineTask.begin();
}

void loop() {
    straightLineTask.update();
    delay(10);
}