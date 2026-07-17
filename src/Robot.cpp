#include "Robot.hpp"

#include <math.h>

namespace {

constexpr float HEADING_KP = 4.0f;
constexpr float WALL_KP = 1.5f;
constexpr float TURN_KP = 2.5f;
constexpr float WALL_TOLERANCE_MM = 5.0f;
constexpr float TURN_TOLERANCE_DEG = 2.0f;
constexpr float MIN_DRIVE_PWM = 65.0f;
constexpr float MAX_DRIVE_PWM = 120.0f;
constexpr float MAX_HEADING_CORRECTION = 50.0f;

float applyMinimumMagnitude(float value, float minimum) {
    if (value > 0.0f && value < minimum) {
        return minimum;
    }
    if (value < 0.0f && value > -minimum) {
        return -minimum;
    }
    return value;
}

}  // namespace

namespace mtrn3100 {

Robot::Robot(Motor& left_motor,
             Motor& right_motor,
             Encoder& left_encoder,
             Encoder& right_encoder,
             Lidar& front_lidar,
             Lidar& left_lidar,
             Lidar& right_lidar,
             IMU& imu,
             float wheel_radius_mm)
    : left_motor_(left_motor),
      right_motor_(right_motor),
      left_encoder_(left_encoder),
      right_encoder_(right_encoder),
      front_lidar_(front_lidar),
      left_lidar_(left_lidar),
      right_lidar_(right_lidar),
      imu_(imu),
      wheel_radius_mm_(wheel_radius_mm) {}

void Robot::begin() {
    left_motor_.begin();
    right_motor_.begin();
    left_encoder_.begin();
    right_encoder_.begin();
    beginSensors();
    imu_.begin();
    imu_.zeroYaw();
    stop();
}

void Robot::beginSensors() {
    front_lidar_.shutdown();
    left_lidar_.shutdown();
    right_lidar_.shutdown();
    delay(20);

    front_lidar_.begin();
    left_lidar_.begin();
    right_lidar_.begin();
}

void Robot::update() {
    imu_.update();

    switch (task_) {
        case Task::STRAIGHT_LINE:
            updateStraightLine();
            break;
        case Task::WALL_DISTANCE:
            updateWallDistance();
            break;
        case Task::TURN:
            updateTurn();
            break;
        case Task::COMMAND_STRING:
            updateCommandString();
            break;
        case Task::IDLE:
        default:
            stopMotors();
            break;
    }
}

void Robot::stop() {
    stopMotors();
    task_ = Task::IDLE;
    finished_ = true;
}

void Robot::startStraightLine(float distance_mm, int16_t pwm) {
    task_ = Task::STRAIGHT_LINE;
    finished_ = false;
    target_distance_mm_ = distance_mm;
    const int32_t pwm_magnitude =
        pwm < 0 ? -static_cast<int32_t>(pwm) : static_cast<int32_t>(pwm);
    base_pwm_ = static_cast<int16_t>(
        constrain(pwm_magnitude, static_cast<int32_t>(0),
                  static_cast<int32_t>(255)));
    start_left_rotation_ = left_encoder_.getRotation();
    start_right_rotation_ = right_encoder_.getRotation();
    target_yaw_deg_ = imu_.getYawDeg();

    Serial.print(F("Straight line target yaw: "));
    Serial.println(target_yaw_deg_);
}

void Robot::startWallDistance(float front_distance_mm) {
    task_ = Task::WALL_DISTANCE;
    finished_ = false;
    target_wall_distance_mm_ = front_distance_mm;
    target_yaw_deg_ = imu_.getYawDeg();

    Serial.print(F("Wall distance target yaw: "));
    Serial.println(target_yaw_deg_);
}

void Robot::startTurn(float angle_deg) {
    task_ = Task::TURN;
    finished_ = false;
    turn_hold_enabled_ = false;
    target_yaw_deg_ = IMU::wrapAngleDeg(imu_.getYawDeg() + angle_deg);
}

void Robot::startTurnHold(float angle_deg) {
    task_ = Task::TURN;
    finished_ = false;
    turn_hold_enabled_ = true;
    target_yaw_deg_ = IMU::wrapAngleDeg(imu_.getYawDeg() + angle_deg);

    Serial.print(F("Turn hold target yaw: "));
    Serial.println(target_yaw_deg_);
}

void Robot::enableLidarCentering(bool enabled,
                                 float target_side_distance_mm) {
    lidar_centering_enabled_ = enabled;
    side_wall_target_mm_ = target_side_distance_mm;
}

void Robot::enableFrontLidarSafety(bool enabled,
                                   uint16_t stop_distance_mm) {
    front_lidar_safety_enabled_ = enabled;
    front_stop_distance_mm_ = stop_distance_mm;
}

bool Robot::isFinished() const {
    return finished_;
}

void Robot::updateStraightLine() {
    const float distance_mm = getAverageDistanceMM();
    if (distance_mm >= target_distance_mm_) {
        finishTask();
        return;
    }

    const float current_yaw = imu_.getYawDeg();
    const float heading_error =
        IMU::wrapAngleDeg(current_yaw - target_yaw_deg_);
    const float correction = headingCorrection(heading_error);
    const float left_pwm = constrain(base_pwm_ + correction, 80.0f, 180.0f);
    const float right_pwm = constrain(base_pwm_ - correction, 80.0f, 180.0f);

    setDrivePWM(left_pwm, right_pwm);

    Serial.print(F("Yaw: "));
    Serial.print(current_yaw);
    Serial.print(F(" Error: "));
    Serial.print(heading_error);
    Serial.print(F(" Distance: "));
    Serial.print(distance_mm);
    Serial.print(F(" L_PWM: "));
    Serial.print(left_pwm);
    Serial.print(F(" R_PWM: "));
    Serial.println(right_pwm);
}

void Robot::updateWallDistance() {
    const uint16_t distance_mm = front_lidar_.readDistance();
    Serial.print(F("Front lidar distance: "));
    Serial.print(distance_mm);
    Serial.println(F(" mm"));

    if (!front_lidar_.isReady() || front_lidar_.timedOut() ||
        distance_mm == 0) {
        stopMotors();
        Serial.println(F("Invalid lidar reading. Stopping."));
        return;
    }

    const float error_mm =
        static_cast<float>(distance_mm) - target_wall_distance_mm_;
    if (fabs(error_mm) <= WALL_TOLERANCE_MM) {
        stopMotors();
        Serial.println(F("At target wall distance."));
        return;
    }

    float pwm = constrain(WALL_KP * error_mm, -MAX_DRIVE_PWM, MAX_DRIVE_PWM);
    pwm = applyMinimumMagnitude(pwm, MIN_DRIVE_PWM);

    const float current_yaw = imu_.getYawDeg();
    const float heading_error =
        IMU::wrapAngleDeg(current_yaw - target_yaw_deg_);
    const float correction = headingCorrection(heading_error);
    const float left_pwm = constrain(pwm + correction, -180.0f, 180.0f);
    const float right_pwm = constrain(pwm - correction, -180.0f, 180.0f);

    setDrivePWM(left_pwm, right_pwm);

    Serial.print(F("Wall error: "));
    Serial.print(error_mm);
    Serial.print(F(" Yaw: "));
    Serial.print(current_yaw);
    Serial.print(F(" Heading error: "));
    Serial.print(heading_error);
    Serial.print(F(" L_PWM: "));
    Serial.print(left_pwm);
    Serial.print(F(" R_PWM: "));
    Serial.println(right_pwm);
}

void Robot::updateTurn() {
    const float current_yaw = imu_.getYawDeg();
    const float yaw_error =
        IMU::wrapAngleDeg(target_yaw_deg_ - current_yaw);

    Serial.print(F("Yaw: "));
    Serial.print(current_yaw);
    Serial.print(F(" Target: "));
    Serial.print(target_yaw_deg_);
    Serial.print(F(" Error: "));
    Serial.println(yaw_error);

    if (fabs(yaw_error) <= TURN_TOLERANCE_DEG) {
        stopMotors();
        finished_ = true;
        if (!turn_hold_enabled_) {
            finishTask();
        }
        return;
    }

    finished_ = false;
    const float pwm = turnPwm(yaw_error);
    setDrivePWM(-pwm, pwm);
}

float Robot::getAverageDistanceMM() const {
    const float left_rotation =
        left_encoder_.getRotation() - start_left_rotation_;
    const float right_rotation =
        right_encoder_.getRotation() - start_right_rotation_;
    return ((fabs(left_rotation) + fabs(right_rotation)) * 0.5f) *
           wheel_radius_mm_;
}

float Robot::headingCorrection(float heading_error_deg) {
    return constrain(HEADING_KP * heading_error_deg,
                     -MAX_HEADING_CORRECTION,
                     MAX_HEADING_CORRECTION);
}

float Robot::turnPwm(float yaw_error_deg) {
    const float pwm = constrain(
        TURN_KP * yaw_error_deg, -MAX_DRIVE_PWM, MAX_DRIVE_PWM);
    return applyMinimumMagnitude(pwm, MIN_DRIVE_PWM);
}

void Robot::setDrivePWM(float left_pwm, float right_pwm) {
    // The motors face opposite directions in the chassis.
    left_motor_.setPWM(
        static_cast<int16_t>(constrain(-left_pwm, -255.0f, 255.0f)));
    right_motor_.setPWM(
        static_cast<int16_t>(constrain(right_pwm, -255.0f, 255.0f)));
}

void Robot::stopMotors() {
    left_motor_.setPWM(0);
    right_motor_.setPWM(0);
}

void Robot::finishTask() {
    stop();
}

}  // namespace mtrn3100
