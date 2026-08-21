#include <Arduino.h>
#include <Wire.h>
#include <U8x8lib.h>

#include "Motor.hpp"
#include "Encoder.hpp"
#include "IMU.hpp"
#include "Lidar.hpp"
#include "Robot.hpp"
#include "AutonomousMapping.hpp"


using namespace mtrn3100;

// One ratio for all straight-line and turning motor output.
// 1.0f is the original output; 0.85f is a conservative slowdown.
constexpr float MOTION_SPEED_SCALE = 0.85f;

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
    16.0   // wheel radius in mm
);

// 4.3
AutonomousMapping autonomousMapping(robot);

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

void setup() {
    delay(1000);

#if LIDAR_DIAGNOSTICS
    Serial.begin(115200);
#endif

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
        oled.clearDisplay();
        oled.setFont(u8x8_font_chroma48medium8_r);
        oled.drawString(0, 0, "Micromouse");
        oled.drawString(0, 1, "Initialising...");
    }

    robot.setMotionSpeedScale(MOTION_SPEED_SCALE);
    robot.begin();

    if (oledReady) {
        oled.clearDisplay();
        oled.drawString(0, 0, "Micromouse");
        oled.drawString(0, 1, "Ready!!!");
    }

    delay(1000);

    // Side LiDAR collision avoidance engages inside 40 mm; when both corridor
    // walls are visible, the gentler centre-line correction also remains active.
    robot.enableSideLidarAvoidance(true, 50.0);

    // Task 4.3: explore the full maze, return to the start through DFS
    // backtracking, then run the calculated shortest route to the goal.
    autonomousMapping.begin(
        1,                              // start row
        3,                              // start column (first usable top cell)
        AutonomousMapping::SOUTH,       // start direction
        1,                              // goal row
        4                               // goal column
    );

    if (oledReady) {
        autonomousMapping.renderToOled(oled);
    }

}

void loop() {
    autonomousMapping.update();
    if (oledReady && autonomousMapping.displayNeedsUpdate()) {
        autonomousMapping.renderToOled(oled);
    }
    delay(10);
}
