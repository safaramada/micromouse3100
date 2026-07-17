#include "Motor.hpp"

namespace mtrn3100 {

Motor::Motor(uint8_t pwm_pin, uint8_t direction_pin)
    : pwm_pin_(pwm_pin), direction_pin_(direction_pin) {}

void Motor::begin() {
    pinMode(pwm_pin_, OUTPUT);
    pinMode(direction_pin_, OUTPUT);
    setPWM(0);
}

void Motor::setPWM(int16_t pwm) {
    pwm = constrain(pwm, -255, 255);
    const bool forward = pwm >= 0;
    const uint8_t duty_cycle = static_cast<uint8_t>(forward ? pwm : -pwm);

    digitalWrite(direction_pin_, forward ? HIGH : LOW);
    analogWrite(pwm_pin_, duty_cycle);
}

}  // namespace mtrn3100
