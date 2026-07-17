#include "Display.hpp"

#include <Wire.h>

namespace mtrn3100 {

bool Display::begin() {
    const uint8_t address = findAddress();
    if (address == 0) {
        Serial.println(F("OLED not found at 0x3C or 0x3D"));
        printI2cDevices();
        return false;
    }

    // U8x8 expects the 8-bit form of the I2C address.
    oled_.setI2CAddress(address << 1);
    oled_.begin();
    oled_.setPowerSave(0);
    oled_.setFont(u8x8_font_chroma48medium8_r);
    ready_ = true;

    Serial.print(F("OLED found at 0x"));
    Serial.println(address, HEX);
    return true;
}

void Display::showInitialising() {
    if (!ready_) {
        return;
    }

    oled_.clearDisplay();
    oled_.drawString(0, 0, "Micromouse");
    oled_.drawString(0, 1, "Initialising...");
}

void Display::showReady() {
    if (!ready_) {
        return;
    }

    oled_.clearDisplay();
    oled_.drawString(0, 0, "Micromouse");
    oled_.drawString(0, 1, "Ready!");
}

bool Display::devicePresent(uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

uint8_t Display::findAddress() {
    constexpr uint8_t ADDRESSES[] = {0x3C, 0x3D};
    for (const uint8_t address : ADDRESSES) {
        if (devicePresent(address)) {
            return address;
        }
    }

    return 0;
}

void Display::printI2cDevices() {
    Serial.println(F("I2C devices found:"));
    bool found_any = false;

    for (uint8_t address = 1; address < 127; ++address) {
        if (!devicePresent(address)) {
            continue;
        }

        Serial.print(F("  0x"));
        if (address < 0x10) {
            Serial.print('0');
        }
        Serial.println(address, HEX);
        found_any = true;
    }

    if (!found_any) {
        Serial.println(F("  none"));
    }
}

}  // namespace mtrn3100
