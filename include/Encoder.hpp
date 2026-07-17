#pragma once

#include <Arduino.h>

namespace mtrn3100 {

class Encoder {
public:
    Encoder(uint8_t channel_a_pin,
            uint8_t channel_b_pin,
            uint16_t counts_per_revolution = 700,
            bool inverted = false);

    void begin();
    void reset();
    int32_t getCount() const;
    float getRotation() const;

private:
    void readEncoder();
    static void readEncoderISR0();
    static void readEncoderISR1();

    const uint8_t channel_a_pin_;
    const uint8_t channel_b_pin_;
    const uint16_t counts_per_revolution_;
    volatile int32_t count_ = 0;
    const bool inverted_;

    static Encoder* instances_[2];
    static uint8_t instance_count_;
};

}  // namespace mtrn3100
