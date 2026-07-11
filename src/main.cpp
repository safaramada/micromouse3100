#include <Arduino.h>

#include "Encoder.hpp"
#include "IMU.hpp"
#include "Lidar.hpp"
#include "Motor.hpp"
#include "Robot.hpp"

#define RUN_IDLE 0
#define RUN_STRAIGHT_LINE 1
#define RUN_WALL_DISTANCE 2
#define RUN_TURN 3
#define RUN_COMMAND_STRING 4

#define ACTIVE_TASK RUN_COMMAND_STRING

#define LEFT_MOTOR_PWM 9 // PIN 9 is a PWM pin
#define LEFT_MOTOR_DIR 10
#define RIGHT_MOTOR_PWM 11 // TODO: Update this pin to match your robot
#define RIGHT_MOTOR_DIR 12 // TODO: Update this pin to match your robot
mtrn3100::Motor left_motor(LEFT_MOTOR_PWM, LEFT_MOTOR_DIR);
mtrn3100::Motor right_motor(RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIR);




#define LEFT_EN_A 2 // PIN 2 is an interrupt
#define LEFT_EN_B 7
#define RIGHT_EN_A 3 // PIN 3 is an interrupt
#define RIGHT_EN_B 8 // TODO: Update this pin to match your robot
mtrn3100::Encoder left_encoder(LEFT_EN_A, LEFT_EN_B);
mtrn3100::Encoder right_encoder(RIGHT_EN_A, RIGHT_EN_B);





mtrn3100::Lidar front_lidar;
mtrn3100::Lidar left_lidar;
mtrn3100::Lidar right_lidar;
mtrn3100::IMU imu;


mtrn3100::Robot robot(left_motor, right_motor, left_encoder, right_encoder, front_lidar, left_lidar, right_lidar, imu);

const char command_string[] = "lfrfflfr";


void setup() {
  Serial.begin(9600);
  robot.begin();

#if ACTIVE_TASK == RUN_STRAIGHT_LINE
  robot.startStraightLine(1000.0); // Drive 1m for Task 3.1
#elif ACTIVE_TASK == RUN_WALL_DISTANCE
  robot.startWallDistance(100.0); // Stop 100mm from the wall for Task 3.2
#elif ACTIVE_TASK == RUN_TURN
  robot.startTurn(-90.0); // Clockwise 90 degree turn for Task 3.3
#elif ACTIVE_TASK == RUN_COMMAND_STRING
  robot.startCommandString(command_string); // Chained movement for Task 3.4
#else
  robot.stop();
#endif
}

void loop() {
  robot.update();
}
