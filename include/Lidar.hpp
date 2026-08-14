#pragma once

// AI-assisted implementation: VL6180X lidar wrapper for returning distance
// readings in millimetres.

#include <Arduino.h>
#include <VL6180X.h>
#include <Wire.h>

namespace mtrn3100 {

class Lidar {
public:

    // Default address for I2C
    static constexpr uint8_t DEFAULT_ADDRESS = 0x29;
    static constexpr uint8_t NO_XSHUT_PIN = 255;


    // Constructor for Lidar class
    Lidar(uint8_t address = DEFAULT_ADDRESS, uint8_t xshut_pin = NO_XSHUT_PIN)
        : address(address), xshut_pin(xshut_pin) {}

    
    
    // Call this once in setup before reading distance.
    void begin() {

        // Initialize I2C communication
        Wire.begin();

        // Check which pin is using rn
        if (xshut_pin != NO_XSHUT_PIN) {
            pinMode(xshut_pin, OUTPUT);
            digitalWrite(xshut_pin, HIGH);
            delay(10);
        }

        // Set the timeout for the sensor
        sensor.setTimeout(timeout_ms);
        sensor.init();
        sensor.configureDefault();

        // Set the I2C address if it's not the default
        if (address != DEFAULT_ADDRESS) {
            sensor.setAddress(address);
        }

        // The library records the status of the most recent I2C transaction.
        ready = (sensor.last_status == 0);
    }

    // Returns the measured distance in millimetres, or 0 when the latest
    // measurement is invalid. Invalid samples are never replaced with an old
    // cached distance.
    uint16_t readDistance() {
        reading_valid = false;

        if (!ready) {
            return 0;
        }

        uint16_t reading = sensor.readRangeSingleMillimeters();
        if (sensor.timeoutOccurred()) {
            return 0;
        }

        uint8_t range_status = sensor.readRangeStatus();

        if (sensor.last_status != 0 ||
            range_status != 0 ||
            reading == 0 ||
            reading > max_valid_distance_mm) {
            return 0;
        }

        reading_valid = true;
        return reading;
    }

    /* Dont need it now */
    // void shutdown() {
    //     if (xshut_pin == NO_XSHUT_PIN) {
    //         return;
    //     }

    //     pinMode(xshut_pin, OUTPUT);
    //     digitalWrite(xshut_pin, LOW);
    //     ready = false;
    //     delay(10);
    // }

    // void wake() {
    //     if (xshut_pin == NO_XSHUT_PIN) {
    //         return;
    //     }

    //     pinMode(xshut_pin, OUTPUT);
    //     digitalWrite(xshut_pin, HIGH);
    //     delay(10);
    // }


    bool isReadingValid() {
        return ready && reading_valid;
    }


public:
    const uint8_t address;
    const uint8_t xshut_pin;

private:
    VL6180X sensor;
    bool ready = false;
    bool reading_valid = false;
    static constexpr uint16_t max_valid_distance_mm = 200;
    static constexpr uint16_t timeout_ms = 100;
};

}  // namespace mtrn3100
