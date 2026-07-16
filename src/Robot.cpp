#include "Robot.hpp"

namespace mtrn3100 {

Robot::Robot(Motor& left_motor,
             Motor& right_motor,
             Encoder& left_encoder,
             Encoder& right_encoder,
             Lidar& front_lidar,
             Lidar& left_lidar,
             Lidar& right_lidar,
             IMU& imu,
             float wheel_radius_mm,
             float wheel_base_mm)
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
      heading_pid(4.0, 0.0, 0.0),
      distance_pid(2.0, 0.0, 0.0),
      turn_pid(4.0, 0.0, 0.0) {}

void Robot::begin() {
    left_encoder.begin();
    right_encoder.begin();

    beginSensors();

    imu.begin();
    imu.zeroYaw();

    stop();
}

void Robot::beginSensors() {
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

void Robot::update() {
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

bool Robot::isFinished() {
    return finished;
}

void Robot::stop() {
    stopMotors();
    task = TASK_IDLE;
    finished = true;
}

float Robot::getAverageDistanceMM() {
    float left_rotation = left_encoder.getRotation() - start_left_rotation;
    float right_rotation = right_encoder.getRotation() - start_right_rotation;

    float average_rotation = (fabs(left_rotation) + fabs(right_rotation)) / 2.0;

    return average_rotation * wheel_radius_mm;
}

void Robot::setDrivePWM(float left_pwm, float right_pwm) {
    left_motor.setPWM(static_cast<int16_t>(constrain(-left_pwm, -255, 255)));

    // Right motor is inverted because negative PWM was forward for your robot
    right_motor.setPWM(static_cast<int16_t>(constrain(right_pwm, -255, 255)));
}

void Robot::stopMotors() {
    left_motor.setPWM(0);
    right_motor.setPWM(0);
}

void Robot::finishTask() {
    stop();
    finished = true;
}

}  // namespace mtrn3100
