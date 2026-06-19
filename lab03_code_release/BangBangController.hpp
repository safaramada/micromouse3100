#pragma once

#include <math.h>

namespace mtrn3100 {

class BangBangController {
public:
    BangBangController(float speed, float deadband) : speed(speed), deadband(deadband) {}

    // Compute the output signal required from the current/actual value.
    float compute(float input) {
        error = setpoint - (input - zero_ref);

        // TODO: IMPLIMENT BANG BANG CONTROLLER - REFER TO THE TUTORIAL SLIDES
        output = 0;

        return output;
    }

    // Function used to return the last calculated error. 
    // The error is the difference between the desired position and current position. 
    float getError() {
      return error;
    }

    // Setting function used to update internal parameters
    void tune(float speed, float deadband) {
      speed = speed;
      deadband = deadband;
    }

    // This must be called before trying to achieve a setpoint.
    // First argument becomes the new zero reference point.
    // Target is the setpoint value.
    void zeroAndSetTarget(float zero, float target) {
        zero_ref = zero;
        setpoint = target;
    }

private:
    float speed, deadband;
    float error, output;
    float setpoint = 0;
    float zero_ref = 0;
};

}  // namespace mtrn3100
