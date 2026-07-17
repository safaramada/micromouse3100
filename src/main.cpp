#include <Arduino.h>
#include <Wire.h>
#include <U8x8lib.h>

#include "Motor.hpp"
#include "Encoder.hpp"
#include "IMU.hpp"
#include "Lidar.hpp"
#include "Robot.hpp"
#include "StraightLineTracking.hpp"
#include "Turning.hpp"


using namespace mtrn3100;

U8X8_SSD1306_128X64_NONAME_HW_I2C oled(U8X8_PIN_NONE);
bool oledReady = false;

// Replace these pins with your real wiring
Motor leftMotor(11, 12);
Motor rightMotor(9, 10);

// Encoder channel A pins should be interrupt-capable pins
Encoder leftEncoder(2, 4, 700, false);
Encoder rightEncoder(3, 5, 700, false);


#define FRONT_XSHUT A2
#define LEFT_XSHUT  A0
#define RIGHT_XSHUT A1

mtrn3100::Lidar front_lidar(0x30, FRONT_XSHUT);
mtrn3100::Lidar left_lidar(0x31, LEFT_XSHUT);
mtrn3100::Lidar right_lidar(0x32, RIGHT_XSHUT);

IMU imu;

Robot robot(
    leftMotor,
    rightMotor,
    leftEncoder,
    rightEncoder,
    front_lidar,
    left_lidar,
    right_lidar,
    imu,
    16.0,  // wheel radius in mm
    80.0   // wheel base in mm
);

StraightLineTracking straightLineTask(robot);
Turning turningTask(robot);

const char command_string[] = "frfrfflflff";

bool i2cDevicePresent(uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

uint8_t findOledAddress() {
    constexpr uint8_t OLED_ADDRESSES[] = {0x3C, 0x3D};

    for (uint8_t address : OLED_ADDRESSES) {
        if (i2cDevicePresent(address)) {
            return address;
        }
    }

    return 0;
}

void printI2cDevices() {
    Serial.println(F("I2C devices found:"));
    bool foundAny = false;

    for (uint8_t address = 1; address < 127; address++) {
        if (i2cDevicePresent(address)) {
            Serial.print(F("  0x"));
            if (address < 0x10) {
                Serial.print('0');
            }
            Serial.println(address, HEX);
            foundAny = true;
        }
    }

    if (!foundAny) {
        Serial.println(F("  none"));
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Wire.begin();
    const uint8_t oledAddress = findOledAddress();

    if (oledAddress != 0) {
        // U8x8 expects the 8-bit form of the I2C address.
        oled.setI2CAddress(oledAddress << 1);
        oled.begin();
        oled.setPowerSave(0);
        oledReady = true;
    }

    if (oledReady) {
        Serial.print(F("OLED found at 0x"));
        Serial.println(oledAddress, HEX);
        oled.clearDisplay();
        oled.setFont(u8x8_font_chroma48medium8_r);
        oled.drawString(0, 0, "Micromouse");
        oled.drawString(0, 1, "Initialising...");
    } else {
        Serial.println(F("OLED not found at 0x3C or 0x3D"));
        printI2cDevices();
    }

    robot.begin();

    if (oledReady) {
        oled.clearDisplay();
        oled.drawString(0, 0, "Micromouse");
        oled.drawString(0, 1, "Ready!");
    }

    delay(1000);

    // Use the side LiDARs to help keep the robot centred
    // robot.enableLidarCentering(true);

    // // Stop if the front wall is closer than 40 mm
    // robot.enableFrontLidarSafety(true, 40);

    // robot.startCommandString(command_string);
    // robot.startTurnHold(-90.0);
    // since its from the bottom, plus 2cm so its plus 20cm
    // robot.startWallDistance(120);
    // robot.startStraightLine(1000);

}

void loop() {
    robot.update();
    delay(10);
}
