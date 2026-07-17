#pragma once

#include <Arduino.h>

namespace mtrn3100 {

class IMU {
public:
    explicit IMU(uint8_t address = 0x68);

    void begin();
    void update();
    void zeroYaw();

    float getYawDeg() const;
    static float wrapAngleDeg(float angle_deg);

private:
    static constexpr float GYRO_SENSITIVITY_LSB_PER_DPS = 32.8f;
    static constexpr uint16_t CALIBRATION_SAMPLES = 500;

    void calibrateGyro();
    float readGyroZ();
    int16_t readRawGyroZ();
    void writeRegister(uint8_t reg, uint8_t value);

    const uint8_t address_;
    float yaw_deg_ = 0.0f;
    float yaw_zero_deg_ = 0.0f;
    float gyro_z_offset_dps_ = 0.0f;
    uint32_t previous_time_us_ = 0;
};

}  // namespace mtrn3100
