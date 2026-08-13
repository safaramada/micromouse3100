#pragma once

#include <Arduino.h>

namespace mtrn3100 {

class Encoder {
public:
    Encoder(uint8_t enc1, uint8_t enc2, uint16_t counts_per_rev = 700, bool invert = false)
        : encoder1_pin(enc1),
          encoder2_pin(enc2),
          counts_per_revolution(counts_per_rev),
          invert(invert) {}

    void begin() {
        pinMode(encoder1_pin, INPUT_PULLUP);
        pinMode(encoder2_pin, INPUT_PULLUP);

        if (instance_count == 0) {
            instances[0] = this;
            attachInterrupt(digitalPinToInterrupt(encoder1_pin), readEncoderISR0, RISING);
            instance_count++;
        } else if (instance_count == 1) {
            instances[1] = this;
            attachInterrupt(digitalPinToInterrupt(encoder1_pin), readEncoderISR1, RISING);
            instance_count++;
        }
    }

    void readEncoder() {
        int step = digitalRead(encoder2_pin) ? 1 : -1;

        if (invert) {
            step = -step;
        }

        count += step;
    }

    void reset() {
        noInterrupts();
        count = 0;
        interrupts();
    }

    long getCount() {
        noInterrupts();
        long temp = count;
        interrupts();
        return temp;
    }

    float getRotation() {
        long temp = getCount();

        if (counts_per_revolution == 0) {
            return 0;
        }

        return (2.0 * PI * temp) / counts_per_revolution;
    }

private:
    static void readEncoderISR0() {
        if (instances[0] != nullptr) {
            instances[0]->readEncoder();
        }
    }

    static void readEncoderISR1() {
        if (instances[1] != nullptr) {
            instances[1]->readEncoder();
        }
    }

public:
    const uint8_t encoder1_pin;
    const uint8_t encoder2_pin;

    uint16_t counts_per_revolution;
    volatile long count = 0;

private:
    bool invert = false;

    static Encoder* instances[2];
    static uint8_t instance_count;
};

}  // namespace mtrn3100
