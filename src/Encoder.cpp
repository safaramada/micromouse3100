#include "Encoder.hpp"

#include <util/atomic.h>

namespace mtrn3100 {

Encoder* Encoder::instances_[2] = {nullptr, nullptr};
uint8_t Encoder::instance_count_ = 0;

Encoder::Encoder(uint8_t channel_a_pin,
                 uint8_t channel_b_pin,
                 uint16_t counts_per_revolution,
                 bool inverted)
    : channel_a_pin_(channel_a_pin),
      channel_b_pin_(channel_b_pin),
      counts_per_revolution_(counts_per_revolution),
      inverted_(inverted) {}

void Encoder::begin() {
    pinMode(channel_a_pin_, INPUT_PULLUP);
    pinMode(channel_b_pin_, INPUT_PULLUP);

    if (instance_count_ >= 2) {
        return;
    }

    const uint8_t slot = instance_count_++;
    instances_[slot] = this;
    attachInterrupt(digitalPinToInterrupt(channel_a_pin_),
                    slot == 0 ? readEncoderISR0 : readEncoderISR1,
                    RISING);
}

void Encoder::reset() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        count_ = 0;
    }
}

int32_t Encoder::getCount() const {
    int32_t count;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        count = count_;
    }
    return count;
}

float Encoder::getRotation() const {
    if (counts_per_revolution_ == 0) {
        return 0.0f;
    }

    return (2.0f * PI * getCount()) / counts_per_revolution_;
}

void Encoder::readEncoder() {
    int8_t step = digitalRead(channel_b_pin_) ? 1 : -1;
    if (inverted_) {
        step = -step;
    }
    count_ += step;
}

void Encoder::readEncoderISR0() {
    if (instances_[0] != nullptr) {
        instances_[0]->readEncoder();
    }
}

void Encoder::readEncoderISR1() {
    if (instances_[1] != nullptr) {
        instances_[1]->readEncoder();
    }
}

}  // namespace mtrn3100
