#pragma once

#include <Arduino.h>
#include <Wire.h>

namespace mtrn3100 {

class IMU {
public:
    IMU(uint8_t address = 0x68) : address(address) {}

    void begin() {
        Wire.begin();

        // Wake up MPU6050
        writeRegister(0x6B, 0x00);
        delay(100);

        // Gyro range: +/- 1000 deg/s.
        // A pickup disturbance can easily exceed the previous +/-250 deg/s
        // range, which made the integrated yaw smaller than the real turn.
        writeRegister(0x1B, 0x10);
        delay(100);

        calibrateGyro();

        yaw_deg = 0;
        yaw_zero_deg = 0;
        prev_time = micros();
    }

    void update() {
        curr_time = micros();
        dt = static_cast<float>(curr_time - prev_time) / 1e6;
        prev_time = curr_time;

        if (dt <= 0) {
            dt = 0.001;
        }

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
    // MPU6050 sensitivity at the configured +/-1000 deg/s range.
    static constexpr float gyroSensitivityLSBPerDPS() {
        return 32.8f;
    }

    void calibrateGyro() {
        long sum = 0;
        const int samples = 500;

        for (int i = 0; i < samples; i++) {
            sum += readRawGyroZ();
            delay(2);
        }

        float raw_offset = static_cast<float>(sum) / samples;
        gyro_z_offset_dps = raw_offset / gyroSensitivityLSBPerDPS();

    }

    float readGyroZ() {
        int16_t raw = readRawGyroZ();

        return static_cast<float>(raw) / gyroSensitivityLSBPerDPS();
    }

    int16_t readRawGyroZ() {
        Wire.beginTransmission(address);
        Wire.write(0x47); // GYRO_ZOUT_H
        Wire.endTransmission(false);

        Wire.requestFrom(address, static_cast<uint8_t>(2));

        if (Wire.available() < 2) {
            return 0;
        }

        int16_t high = Wire.read();
        int16_t low = Wire.read();

        return (high << 8) | low;
    }

    void writeRegister(uint8_t reg, uint8_t value) {
        Wire.beginTransmission(address);
        Wire.write(reg);
        Wire.write(value);
        Wire.endTransmission();
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

}
