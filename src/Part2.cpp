#include "Robot.hpp"

namespace mtrn3100 {

void Robot::startWallDistance(float front_distance_mm) {
    task = TASK_WALL_DISTANCE;
    finished = false;
    target_wall_distance_mm = front_distance_mm;
    distance_pid.zeroAndSetTarget(0, 0);
}

void Robot::updateWallDistance() {
    uint16_t distance_mm = front_lidar.readDistance();

    Serial.print("Front lidar distance: ");
    Serial.print(distance_mm);
    Serial.println(" mm");

    if (!front_lidar.isReady() || front_lidar.timedOut() || distance_mm == 0) {
        stopMotors();
        Serial.println("Invalid lidar reading. Stopping.");
        return;
    }

    float error_mm = static_cast<float>(distance_mm) - target_wall_distance_mm;

    if (fabs(error_mm) <= wall_tolerance_mm) {
        stopMotors();
        Serial.println("At target wall distance.");
        return;
    }

    float Kp_wall = 1.5;
    float pwm = Kp_wall * error_mm;

    pwm = constrain(pwm, -120, 120);

    if (fabs(pwm) < 65) {
        if (pwm > 0) {
            pwm = 65;
        } else {
            pwm = -65;
        }
    }

    setDrivePWM(pwm, pwm);

    Serial.print("Wall error: ");
    Serial.print(error_mm);
    Serial.print(" PWM: ");
    Serial.println(pwm);
}

}  // namespace mtrn3100
