#pragma once

#include <Arduino.h>
#include <VL6180X.h>

namespace mtrn3100 {

class Lidar {
public:
    static constexpr uint8_t DEFAULT_ADDRESS = 0x29;
    static constexpr uint8_t NO_XSHUT_PIN = UINT8_MAX;

    explicit Lidar(uint8_t address = DEFAULT_ADDRESS,
                   uint8_t xshut_pin = NO_XSHUT_PIN);

    void begin();
    void shutdown();
    uint16_t readDistance();

    bool isReady() const;
    bool timedOut() const;

private:
    static constexpr uint16_t MAX_VALID_DISTANCE_MM = 2000;
    static constexpr uint16_t TIMEOUT_MS = 100;

    VL6180X sensor_;
    const uint8_t address_;
    const uint8_t xshut_pin_;
    uint16_t distance_mm_ = 0;
    bool ready_ = false;
    bool timed_out_ = false;
};

}  // namespace mtrn3100
