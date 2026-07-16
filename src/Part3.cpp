#include "Robot.hpp"

bool turn_waiting = false;
bool turn_correction_started = false;

unsigned long turn_wait_start_ms = 0;

static constexpr unsigned long TURN_WAIT_MS = 2000;



namespace mtrn3100 {

void Robot::startTurn(float angle_deg) {
    task = TASK_TURN;
    finished = false;

    target_yaw_deg =
        IMU::wrapAngleDeg(imu.getYawDeg() + angle_deg);

    turn_pid.zeroAndSetTarget(0, 0);

    // Reset the waiting state for this new turn
    turn_waiting = false;
    turn_correction_started = false;
    turn_wait_start_ms = 0;
}


void Robot::updateTurn() {
    float current_yaw = imu.getYawDeg();

    float yaw_error =
        IMU::wrapAngleDeg(target_yaw_deg - current_yaw);

    Serial.print("Yaw: ");
    Serial.print(current_yaw);
    Serial.print(" Target: ");
    Serial.print(target_yaw_deg);
    Serial.print(" Error: ");
    Serial.println(yaw_error);

    // Target has been reached
    if (fabs(yaw_error) <= turn_tolerance_deg) {
        stopMotors();
        finished = true;

        // Reset so a future turn can pause again
        turn_waiting = false;
        turn_correction_started = false;

        return;
    }

    finished = false;

    /*
     * Begin the waiting period only once.
     */
    if (!turn_waiting && !turn_correction_started) {
        turn_waiting = true;
        turn_wait_start_ms = millis();

        stopMotors();

        Serial.println("Yaw error detected. Waiting before turning...");
        return;
    }

    /*
     * Continue waiting without blocking the whole program.
     */
    if (turn_waiting) {
        stopMotors();

        if (millis() - turn_wait_start_ms < TURN_WAIT_MS) {
            return;
        }

        // Waiting has finished
        turn_waiting = false;
        turn_correction_started = true;

        Serial.println("Wait complete. Starting turn...");
    }

    /*
     * Once correction has started, this section runs continuously.
     * It will not pause again while the yaw remains outside tolerance.
     */
    float Kp_turn = 2.5;
    float pwm = Kp_turn * yaw_error;

    pwm = constrain(pwm, -120, 120);

    // Minimum PWM to overcome motor friction
    if (fabs(pwm) < 45) {
        if (pwm > 0) {
            pwm = 45;
        } else {
            pwm = -45;
        }
    }

    setDrivePWM(-pwm, pwm);
}

}  // namespace mtrn3100
