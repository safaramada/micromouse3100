#pragma once

#include <Arduino.h>
#include <math.h>

namespace mtrn3100 {

class PIDController {
public:
    PIDController(float kp, float ki, float kd)
        : kp(kp), ki(ki), kd(kd) {}

    float compute(float input) {
        curr_time = micros();

        if (prev_time == 0) {
            prev_time = curr_time;
        }

        dt = static_cast<float>(curr_time - prev_time) / 1e6;
        prev_time = curr_time;

        if (dt <= 0) {
            dt = 0.001;
        }

        error = setpoint - (input - zero_ref);

        integral += error * dt;
        derivative = (error - prev_error) / dt;

        output = kp * error + ki * integral + kd * derivative;

        prev_error = error;

        return output;
    }

    void tune(float p, float i, float d) {
        kp = p;
        ki = i;
        kd = d;
    }

    float getError() {
        return error;
    }

    void zeroAndSetTarget(float zero, float target) {
        prev_time = micros();
        zero_ref = zero;
        setpoint = target;

        error = 0;
        prev_error = 0;
        integral = 0;
        derivative = 0;
        output = 0;
    }

public:
    uint32_t prev_time = 0;
    uint32_t curr_time = 0;
    float dt = 0;

private:
    float kp, ki, kd;
    float error = 0;
    float derivative = 0;
    float integral = 0;
    float output = 0;
    float prev_error = 0;
    float setpoint = 0;
    float zero_ref = 0;
};

}