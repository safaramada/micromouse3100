#include "Robot.hpp"

namespace mtrn3100 {

void Robot::startStraightLine(float distance_mm, int16_t pwm) {
    task = TASK_STRAIGHT_LINE;
    finished = false;

    target_distance_mm = distance_mm;
    base_pwm = abs(pwm);

    start_left_rotation = left_encoder.getRotation();
    start_right_rotation = right_encoder.getRotation();

    target_yaw_deg = imu.getYawDeg();

    Serial.print("Straight line target yaw: ");
    Serial.println(target_yaw_deg);
}

void Robot::updateStraightLine() {
    float distance_mm = getAverageDistanceMM();

    if (distance_mm >= target_distance_mm) {
        finishTask();
        return;
    }

    float current_yaw = imu.getYawDeg();

    // Positive if robot has rotated away from starting heading
    float heading_error = IMU::wrapAngleDeg(current_yaw - target_yaw_deg);

    float Kp_heading = 4.0;
    float correction = Kp_heading * heading_error;

    correction = constrain(correction, -50, 50);

    float left_pwm = base_pwm + correction;
    float right_pwm = base_pwm - correction;

    left_pwm = constrain(left_pwm, 80, 180);
    right_pwm = constrain(right_pwm, 80, 180);

    setDrivePWM(left_pwm, right_pwm);

    Serial.print("Yaw: ");
    Serial.print(current_yaw);
    Serial.print(" Error: ");
    Serial.print(heading_error);
    Serial.print(" Distance: ");
    Serial.print(distance_mm);
    Serial.print(" L_PWM: ");
    Serial.print(left_pwm);
    Serial.print(" R_PWM: ");
    Serial.println(right_pwm);
}

}  // namespace mtrn3100
