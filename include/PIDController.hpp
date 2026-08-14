#pragma once

#include <Arduino.h>
#include <math.h>

namespace mtrn3100 {

class PIDController {
public:
    PIDController(float kp, float ki, float kd)
        : kp(kp), ki(ki), kd(kd) {}

    // Use this when the caller already has a signed control error. A positive
    // error produces a positive controller output.
    float computeFromError(float current_error) {
        constexpr float derivative_filter_alpha = 0.25f;
        constexpr float max_derivative_dt = 0.25f;
        constexpr float integral_limit = 500.0f;

        uint32_t curr_time = micros();

        uint32_t elapsed_us = (prev_time == 0) ? 0 : curr_time - prev_time;
        float dt = static_cast<float>(elapsed_us) / 1e6f;
        prev_time = curr_time;

        if (elapsed_us == 0 || dt > max_derivative_dt) {
            dt = (elapsed_us == 0) ? 0.001f : dt;
            derivative = 0;
        } else {
            float raw_derivative = (current_error - prev_error) / dt;
            derivative += derivative_filter_alpha *
                (raw_derivative - derivative);
        }

        integral += current_error * dt;
        integral = constrain(integral, -integral_limit, integral_limit);

        float output =
            kp * current_error + ki * integral + kd * derivative;

        prev_error = current_error;

        return output;
    }

    void tune(float p, float i, float d) {
        kp = p;
        ki = i;
        kd = d;
    }

    void reset(float initial_error = 0) {
        prev_time = micros();
        prev_error = initial_error;
        integral = 0;
        derivative = 0;
    }

private:
    uint32_t prev_time = 0;
    float kp, ki, kd;
    float derivative = 0;
    float integral = 0;
    float prev_error = 0;
};

}
