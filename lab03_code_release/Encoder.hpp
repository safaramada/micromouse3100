#pragma once

#include <Arduino.h>

namespace mtrn3100 {


// The encoder class is a simple interface which counts and stores an encoders count.
// Encoder pin 1 is attached to the interupt on the arduino and used to trigger the count.
// Encoder pin 2 is attached to any digital pin and used to derrive rotation direction.
// The count is stored as a volatile variable due to the high frequency updates. 
class Encoder {
public:
    Encoder(uint8_t enc1, uint8_t enc2) : encoder1_pin(enc1), encoder2_pin(enc2) {
        instance = this;  // Store the instance pointer
        pinMode(encoder1_pin, INPUT_PULLUP);
        pinMode(encoder2_pin, INPUT_PULLUP);

        // TODO: attach the interrupt on pin one such that it calls the readEncoderISR function on a rising edge

    }


    // Encoder function used to update the encoder
    void readEncoder() {
        noInterrupts();

        // NOTE: DO NOT PLACE SERIAL PRINT STATEMENTS IN THIS FUNCTION
        // NOTE: DO NOT CALL THIS FUNCTION MANUALLY IT WILL ONLY WORK IF CALLED BY THE INTERRUPT
        // TODO: Increase or Decrease the count by one based on the reading on encoder pin 2

        interrupts();
    }

    // Helper function which to convert encouder count to radians
    float getRotation() {

        // TODO: Convert encoder count to radians

        return 0;
    }

private:
    static void readEncoderISR() {
        if (instance != nullptr) {
            instance->readEncoder();
        }
    }

public:
    const uint8_t encoder1_pin;
    const uint8_t encoder2_pin;
    volatile int8_t direction;
    float position = 0;
    uint16_t counts_per_revolution = 0; //TODO: Identify how many encoder counts are in one rotation
    volatile long count = 0;
    uint32_t prev_time;
    bool read = false;

private:
    static Encoder* instance;
};

Encoder* Encoder::instance = nullptr;

}  // namespace mtrn3100
