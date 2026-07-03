#pragma once

#include <Arduino.h>
#include <Wire.h>

namespace mtrn3100 {


// The lidar class is a simple interface designed to assist in distance sensing.
// This starter implementation assumes an I2C time-of-flight style lidar sensor.
// You may choose to impliment additional functionality in the future such as
// multi-sensor support, filtering, or wall detection.
class Lidar {
public:
    Lidar(uint8_t address = 0x29) : address(address) {}

    // This function should be called once in setup before reading distance.
    void begin() {
        Wire.begin();

        // TODO: Initialise the lidar sensor or external library here.
        // TODO: Configure timing budget, distance mode, or measurement mode if required.
    }

    // This function returns the measured distance from the lidar sensor.
    // NOTE: Choose and document your distance units. Millimetres are common for lidar sensors.
    uint16_t readDistance() {

        // TODO: Read the sensor using I2C or the lidar library.
        // TODO: Convert the measurement into the chosen units.
        // TODO: Store the latest valid measurement in distance.

        return distance;
    }

    uint16_t getDistance() {
        return distance;
    }

public:
    const uint8_t address;
    uint16_t distance = 0;
};

}  // namespace mtrn3100
