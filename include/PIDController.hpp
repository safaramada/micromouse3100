#pragma once

#include <Arduino.h>
#include <math.h>

namespace mtrn3100 {

class PIDController {
public:
    PIDController(float kp, float ki, float kd)
        : kp(kp), ki(ki), kd(kd) {}

    float compute(float input) {
        return computeFromError(setpoint - (input - zero_ref));
    }

    // Use this when the caller already has a signed control error. A positive
    // error produces a positive controller output.
    float computeFromError(float current_error) {
        constexpr float derivative_filter_alpha = 0.25f;
        constexpr float max_derivative_dt = 0.25f;
        constexpr float integral_limit = 500.0f;

        curr_time = micros();

        uint32_t elapsed_us = (prev_time == 0) ? 0 : curr_time - prev_time;
        dt = static_cast<float>(elapsed_us) / 1e6f;
        prev_time = curr_time;

        error = current_error;

        if (elapsed_us == 0 || dt > max_derivative_dt) {
            dt = (elapsed_us == 0) ? 0.001f : dt;
            derivative = 0;
        } else {
            float raw_derivative = (error - prev_error) / dt;
            derivative += derivative_filter_alpha *
                (raw_derivative - derivative);
        }

        integral += error * dt;
        integral = constrain(integral, -integral_limit, integral_limit);

        output = kp * error + ki * integral + kd * derivative;

        prev_error = error;

        return output;
    }

    void tune(float p, float i, float d) {
        kp = p;
        ki = i;
        kd = d;
    }

    void reset(float initial_error = 0) {
        prev_time = micros();
        curr_time = prev_time;
        dt = 0;
        error = initial_error;
        prev_error = initial_error;
        integral = 0;
        derivative = 0;
        output = 0;
    }

    float getError() {
        return error;
    }

    void zeroAndSetTarget(float zero, float target) {
        zero_ref = zero;
        setpoint = target;
        reset();
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
