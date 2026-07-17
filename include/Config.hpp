#pragma once

#include <Arduino.h>

namespace config {

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint16_t LOOP_DELAY_MS = 10;

constexpr uint8_t LEFT_MOTOR_PWM_PIN = 11;
constexpr uint8_t LEFT_MOTOR_DIRECTION_PIN = 12;
constexpr uint8_t RIGHT_MOTOR_PWM_PIN = 9;
constexpr uint8_t RIGHT_MOTOR_DIRECTION_PIN = 10;

constexpr uint8_t LEFT_ENCODER_A_PIN = 2;
constexpr uint8_t LEFT_ENCODER_B_PIN = 4;
constexpr uint8_t RIGHT_ENCODER_A_PIN = 3;
constexpr uint8_t RIGHT_ENCODER_B_PIN = 5;
constexpr uint16_t ENCODER_COUNTS_PER_REVOLUTION = 700;

constexpr uint8_t FRONT_LIDAR_ADDRESS = 0x30;
constexpr uint8_t LEFT_LIDAR_ADDRESS = 0x31;
constexpr uint8_t RIGHT_LIDAR_ADDRESS = 0x32;
constexpr uint8_t FRONT_LIDAR_XSHUT_PIN = A2;
constexpr uint8_t LEFT_LIDAR_XSHUT_PIN = A0;
constexpr uint8_t RIGHT_LIDAR_XSHUT_PIN = A1;

constexpr float WHEEL_RADIUS_MM = 16.0f;

}  // namespace config
