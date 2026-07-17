#include "IMU.hpp"

#include <Wire.h>

namespace mtrn3100 {

IMU::IMU(uint8_t address) : address_(address) {}

void IMU::begin() {
    // Wake the MPU6050 and select the +/-1000 degrees/second gyro range.
    writeRegister(0x6B, 0x00);
    delay(100);
    writeRegister(0x1B, 0x10);
    delay(100);

    calibrateGyro();
    yaw_deg_ = 0.0f;
    yaw_zero_deg_ = 0.0f;
    previous_time_us_ = micros();
}

void IMU::update() {
    const uint32_t current_time_us = micros();
    float elapsed_seconds =
        static_cast<float>(current_time_us - previous_time_us_) / 1000000.0f;
    previous_time_us_ = current_time_us;

    if (elapsed_seconds <= 0.0f) {
        elapsed_seconds = 0.001f;
    }

    yaw_deg_ += (readGyroZ() - gyro_z_offset_dps_) * elapsed_seconds;
    yaw_deg_ = wrapAngleDeg(yaw_deg_);
}

void IMU::zeroYaw() {
    yaw_zero_deg_ = yaw_deg_;
}

float IMU::getYawDeg() const {
    return wrapAngleDeg(yaw_deg_ - yaw_zero_deg_);
}

float IMU::wrapAngleDeg(float angle_deg) {
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

void IMU::calibrateGyro() {
    int32_t sum = 0;
    Serial.println(F("Calibrating IMU. Keep robot still."));

    for (uint16_t sample = 0; sample < CALIBRATION_SAMPLES; ++sample) {
        sum += readRawGyroZ();
        delay(2);
    }

    const float raw_offset =
        static_cast<float>(sum) / CALIBRATION_SAMPLES;
    gyro_z_offset_dps_ = raw_offset / GYRO_SENSITIVITY_LSB_PER_DPS;

    Serial.print(F("Gyro Z offset: "));
    Serial.println(gyro_z_offset_dps_);
}

float IMU::readGyroZ() {
    return static_cast<float>(readRawGyroZ()) /
           GYRO_SENSITIVITY_LSB_PER_DPS;
}

int16_t IMU::readRawGyroZ() {
    Wire.beginTransmission(address_);
    Wire.write(0x47);  // GYRO_ZOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(address_, static_cast<uint8_t>(2));

    if (Wire.available() < 2) {
        return 0;
    }

    const uint8_t high = Wire.read();
    const uint8_t low = Wire.read();
    return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}

void IMU::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(address_);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

}  // namespace mtrn3100
