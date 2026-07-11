#pragma once

#include <Arduino.h>
#include "Robot.hpp"

namespace mtrn3100 {

class Turning {
public:
    Turning(Robot& robot) : robot(robot) {}

    void begin() {
        /*
          IMPORTANT:
          If clockwise makes your IMU yaw increase, use +90.
          If clockwise makes your IMU yaw decrease, use -90.

          Start with +90. If it turns anticlockwise, change it to -90.
        */
        robot.startTurn(90.0);
    }

    void update() {
        robot.update();
    }

private:
    Robot& robot;
};

}