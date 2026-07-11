#pragma once

#include <Arduino.h>

namespace mtrn3100 {


// The encoder class is a simple interface which counts and stores an encoders count.
// Encoder pin 1 is attached to the interrupt on the arduino and used to trigger the count.
// Encoder pin 2 is attached to any digital pin and used to derive rotation direction.
// The count is stored as a volatile variable due to the high frequency updates. 
class Encoder {
public:
    Encoder(uint8_t enc1, uint8_t enc2);

    // Helper function which to convert encoder count to radians
    long getCount();
    void reset();
    float getRotation();

    const uint8_t encoder1_pin;
    const uint8_t encoder2_pin;
    volatile int8_t direction = 0;
    float position = 0;
    uint16_t counts_per_revolution = 1400; //TODO: Identify how many encoder counts are in one rotation
    volatile long count = 0;
    uint32_t prev_time;
    bool read = false;

private:
    // Encoder function used to update the encoder
    void readEncoder();

    static void leftISR();
    static void rightISR();

    static Encoder* leftEncoder;
    static Encoder* rightEncoder;
};
}  // namespace mtrn3100
