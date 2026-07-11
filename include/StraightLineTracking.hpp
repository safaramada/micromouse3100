#pragma once

#include <Arduino.h>
#include "Robot.hpp"

namespace mtrn3100 {

class StraightLineTracking {
public:
    StraightLineTracking(Robot& robot) : robot(robot) {}

    void begin() {
        // Slightly more than 1m so it clearly passes the finish line
        robot.startStraightLine(1050.0, 140);
    }

    void update() {
        robot.update();
    }

private:
    Robot& robot;
};

}