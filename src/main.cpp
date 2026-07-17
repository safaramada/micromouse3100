#include <Arduino.h>
#include <Wire.h>

#include "Config.hpp"
#include "Display.hpp"
#include "Encoder.hpp"
#include "IMU.hpp"
#include "Lidar.hpp"
#include "Motor.hpp"
#include "Robot.hpp"

namespace {

using mtrn3100::Display;
using mtrn3100::Encoder;
using mtrn3100::IMU;
using mtrn3100::Lidar;
using mtrn3100::Motor;
using mtrn3100::Robot;

Display display;

Motor left_motor(config::LEFT_MOTOR_PWM_PIN,
                 config::LEFT_MOTOR_DIRECTION_PIN);
Motor right_motor(config::RIGHT_MOTOR_PWM_PIN,
                  config::RIGHT_MOTOR_DIRECTION_PIN);

Encoder left_encoder(config::LEFT_ENCODER_A_PIN,
                     config::LEFT_ENCODER_B_PIN,
                     config::ENCODER_COUNTS_PER_REVOLUTION);
Encoder right_encoder(config::RIGHT_ENCODER_A_PIN,
                      config::RIGHT_ENCODER_B_PIN,
                      config::ENCODER_COUNTS_PER_REVOLUTION);

Lidar front_lidar(config::FRONT_LIDAR_ADDRESS,
                  config::FRONT_LIDAR_XSHUT_PIN);
Lidar left_lidar(config::LEFT_LIDAR_ADDRESS,
                 config::LEFT_LIDAR_XSHUT_PIN);
Lidar right_lidar(config::RIGHT_LIDAR_ADDRESS,
                  config::RIGHT_LIDAR_XSHUT_PIN);
IMU imu;

Robot robot(left_motor,
            right_motor,
            left_encoder,
            right_encoder,
            front_lidar,
            left_lidar,
            right_lidar,
            imu,
            config::WHEEL_RADIUS_MM);

}  // namespace

void setup() {
    Serial.begin(config::SERIAL_BAUD);
    delay(1000);

    Wire.begin();
    display.begin();
    display.showInitialising();

    robot.begin();
    display.showReady();
    delay(1000);

    // Select a behavior here when testing:
    // robot.startStraightLine(1000.0f);
    // robot.startWallDistance(120.0f);
    // robot.startTurnHold(-90.0f);



    // robot.enableLidarCentering(true);
    // robot.enableFrontLidarSafety(true, 40);
    robot.startCommandString("frfrfflflff");
}

void loop() {
    robot.update();
    delay(config::LOOP_DELAY_MS);
}
