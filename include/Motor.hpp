#pragma once

#include <Arduino.h>
namespace mtrn3100 {

class Motor {
public:
    Motor(uint8_t pwm_pin, uint8_t direction_pin);

    void begin();
    void setPWM(int16_t pwm);

private:
    const uint8_t pwm_pin_;
    const uint8_t direction_pin_;
};

}  // namespace mtrn3100
