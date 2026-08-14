#include <Arduino.h>

#include "Motor.hpp"
#include "Encoder.hpp"
#include "IMU.hpp"
#include "Lidar.hpp"
#include "Robot.hpp"
// #include "StraightLineTracking.hpp"
// #include "Turning.hpp"


using namespace mtrn3100;

// Replace these pins with your real wiring
Motor leftMotor(11, 12);
Motor rightMotor(9, 10);

// Encoder channel A pins should be interrupt-capable pins
Encoder leftEncoder(2, 4, 700, false);
Encoder rightEncoder(3, 5, 700, false);


#define FRONT_XSHUT A2
#define LEFT_XSHUT  A0
#define RIGHT_XSHUT A1

mtrn3100::Lidar front_lidar(0x30, FRONT_XSHUT);
mtrn3100::Lidar left_lidar(0x31, LEFT_XSHUT);
mtrn3100::Lidar right_lidar(0x32, RIGHT_XSHUT);

IMU imu;

Robot robot(
    leftMotor,
    rightMotor,
    leftEncoder,
    rightEncoder,
    front_lidar,
    left_lidar,
    right_lidar,
    imu,
    16.0,  // wheel radius in mm
    80.0   // wheel base in mm
);

// StraightLineTracking straightLineTask(robot);
// Turning turningTask(robot);

// const char command_string[] = "rfffrf,[(-45.0,145.8),(0.0,138.9),(0.0,145.8),(0.0,138.9),(0.0,145.8),(0.0,138.9),(0.0,145.8),(45.0,0.0)],frf";
//  const char command_string[] = "ffffrfrflf,[(22.2,115.3),(-0.9,119.8),(0.0,119.8),(0.9,115.3),(11.4,104.9),(0.0,104.9),(0.0,104.9),(-33.7,0.0)],frffff";

const char command_string[] = "flf,[(-14.9,75.3),(-3.5,76.7),(9.5,93.3),(5.4,77.8),(3.6,48.5),(50.5,106.8),(11.0,132.4),(-14.6,99.5),(-2.0,96.0),(2.0,99.5),(-47.0,0.0)],f";



















// const char command_string[] =
// "f,[(-45.0,00), f (-45.0,0.0), f,  (-45.0,0.0), f, (-45.0,0.0)]";

void setup() {
    Serial.begin(115200);
    delay(1000);

    robot.begin();

    delay(1000);

    // Centre when both walls are visible and retain close-wall avoidance when
    // only one is visible. The final run before '[' also uses the available
    // side wall as an entrance localization reference.
    robot.enableSideLidarAvoidance(true, 50.0);

    // Start the next turn if the front wall is closer than 50 mm.
    robot.enableFrontLidarSafety(true, 50);

    robot.startCommandString(command_string, 130);
    // robot.startTurnHold(-90.0);
    // since its from the bottom, plus 2cm so its plus 20cm
    // robot.startWallDistance(120);   // DIRIVING AND STOPPING TASK 
    // robot.startStraightLine(3000, 130); // STRAIGHT LINE TRACKING TASK

}

void loop() {
    robot.update();
    delay(10);
}
