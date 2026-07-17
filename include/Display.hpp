#pragma once

#include <Arduino.h>
#include <U8x8lib.h>

namespace mtrn3100 {

class Display {
public:
    bool begin();
    void showInitialising();
    void showReady();

private:
    static bool devicePresent(uint8_t address);
    static uint8_t findAddress();
    static void printI2cDevices();

    U8X8_SSD1306_128X64_NONAME_HW_I2C oled_{U8X8_PIN_NONE};
    bool ready_ = false;
};

}  // namespace mtrn3100
