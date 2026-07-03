#pragma once

#include <Arduino.h>
#include <Wire.h>

namespace mtrn3100 {


// The IMU class is a simple interface designed to assist in heading control.
// This starter implementation assumes an I2C IMU with a z-axis gyroscope.
class IMU {
public:
    IMU(uint8_t address = 0x68) : address(address) {}

    // This function should be called once in setup before reading the IMU.
    void begin() {
        Wire.begin();
        prev_time = micros();

        // TODO: Initialise the IMU sensor or external library here.
        // TODO: Calibrate the gyro offset while the robot is stationary.
    }

    // This function should be called regularly in loop.
    void update() {
        curr_time = micros();
        dt = static_cast<float>(curr_time - prev_time) / 1e6;
        prev_time = curr_time;

        // TODO: Read the z-axis gyro rate from the IMU in degrees per second.
        gyro_z_dps = readGyroZ();

        yaw_deg += (gyro_z_dps - gyro_z_offset_dps) * dt;
        yaw_deg = wrapAngleDeg(yaw_deg);
    }

    void zeroYaw() {
        yaw_zero_deg = yaw_deg;
    }

    float getYawDeg() {
        return wrapAngleDeg(yaw_deg - yaw_zero_deg);
    }

    float getRawYawDeg() {
        return yaw_deg;
    }

    static float wrapAngleDeg(float angle_deg) {
        while (angle_deg > 180.0) {
            angle_deg -= 360.0;
        }

        while (angle_deg < -180.0) {
            angle_deg += 360.0;
        }

        return angle_deg;
    }

private:
    float readGyroZ() {
        // TODO: Replace this with the IMU library call for gyro z rate.
        return 0;
    }

public:
    const uint8_t address;
    float yaw_deg = 0;
    float yaw_zero_deg = 0;
    float gyro_z_dps = 0;
    float gyro_z_offset_dps = 0;
    float dt = 0;
    uint32_t prev_time = 0;
    uint32_t curr_time = 0;
};

}  // namespace mtrn3100
