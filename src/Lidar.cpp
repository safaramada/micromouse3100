#include "Lidar.hpp"

namespace mtrn3100 {

Lidar::Lidar(uint8_t address, uint8_t xshut_pin)
    : address_(address), xshut_pin_(xshut_pin) {}

void Lidar::begin() {
    if (xshut_pin_ != NO_XSHUT_PIN) {
        pinMode(xshut_pin_, OUTPUT);
        digitalWrite(xshut_pin_, HIGH);
        delay(10);
    }

    sensor_.setTimeout(TIMEOUT_MS);
    sensor_.init();
    sensor_.configureDefault();
    if (address_ != DEFAULT_ADDRESS) {
        sensor_.setAddress(address_);
    }
    ready_ = true;
}

void Lidar::shutdown() {
    if (xshut_pin_ != NO_XSHUT_PIN) {
        pinMode(xshut_pin_, OUTPUT);
        digitalWrite(xshut_pin_, LOW);
    }
    ready_ = false;
    timed_out_ = false;
}

uint16_t Lidar::readDistance() {
    if (!ready_) {
        return distance_mm_;
    }

    const uint16_t reading = sensor_.readRangeSingleMillimeters();
    timed_out_ = sensor_.timeoutOccurred();
    if (!timed_out_ && reading > 0 && reading < MAX_VALID_DISTANCE_MM) {
        distance_mm_ = reading;
    }
    return distance_mm_;
}

bool Lidar::isReady() const {
    return ready_;
}

bool Lidar::timedOut() const {
    return timed_out_;
}

}  // namespace mtrn3100
