#pragma once

#include <Arduino.h>
#include <math.h>

namespace mtrn3100 {

class Motor {
public:
    Motor(uint8_t pwm_pin, uint8_t dir_pin)
        : pwm_pin(pwm_pin), dir_pin(dir_pin) {
        pinMode(pwm_pin, OUTPUT);
        pinMode(dir_pin, OUTPUT);

        analogWrite(pwm_pin, 0);
        digitalWrite(dir_pin, LOW);
    }

    void setPWM(int16_t pwm) {
        pwm = constrain(pwm, -255, 255);

        if (pwm >= 0) {
            digitalWrite(dir_pin, HIGH);
            analogWrite(pwm_pin, pwm);
        } else {
            digitalWrite(dir_pin, LOW);
            analogWrite(pwm_pin, -pwm);
        }
    }

private:
    const uint8_t pwm_pin;
    const uint8_t dir_pin;
};

}