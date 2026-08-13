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
#include "AutonomousMapping.hpp"


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

// 4.3
AutonomousMapping autonomousMapping(robot);

uint8_t displayedMappingRow = 0xFF;
uint8_t displayedMappingCol = 0xFF;
bool displayedMazeComplete = false;
AutonomousMapping::DebugStatus displayedDebugStatus =
    AutonomousMapping::STATUS_READY;
bool displayedMazePage = false;

const char STATUS_READY_TEXT[] PROGMEM = "Ready";
const char STATUS_MOVING_TEXT[] PROGMEM = "Moving";
const char STATUS_TURNING_TEXT[] PROGMEM = "Turning";
const char STATUS_FRONT_WALL_TEXT[] PROGMEM = "Front wall";
const char STATUS_SIDE_WALL_TEXT[] PROGMEM = "Side wall";
const char STATUS_TIMEOUT_TEXT[] PROGMEM = "Lidar timeout";
const char STATUS_NO_ROUTE_TEXT[] PROGMEM = "No route";
const char STATUS_COMPLETE_TEXT[] PROGMEM = "Completed";

const char command_string[] = "ffffrfflffrfrflflfrffrffflfrffffrflfrffff";
// const char command_string[] = "ffffrflfrfrflflfrfflfrfrflfrflfrffffrflfrffffflflffff";

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

void updateMappingOled() {
    if (!oledReady) return;

    uint8_t row = autonomousMapping.getCurrentRow();
    uint8_t col = autonomousMapping.getCurrentCol();
    bool complete = autonomousMapping.isComplete();
    AutonomousMapping::DebugStatus status = autonomousMapping.getDebugStatus();
    bool mazePage = ((millis() / 3000UL) % 2) != 0;

    if (row == displayedMappingRow &&
        col == displayedMappingCol &&
        complete == displayedMazeComplete &&
        status == displayedDebugStatus &&
        mazePage == displayedMazePage) {
        return;
    }

    displayedMappingRow = row;
    displayedMappingCol = col;
    displayedMazeComplete = complete;
    displayedDebugStatus = status;
    displayedMazePage = mazePage;

    oled.clearDisplay();

    if (mazePage) {
        const uint8_t mazeSize = autonomousMapping.getMazeSize();
        // U8x8 has eight text rows. Show rows 0-7 normally and scroll down
        // one row when the robot reaches the final maze row.
        uint8_t firstRow = row >= 8 ? 1 : 0;
        char mazeLine[10];
        mazeLine[9] = '\0';

        for (uint8_t displayRow = 0; displayRow < 8; displayRow++) {
            uint8_t mazeRow = firstRow + displayRow;
            for (uint8_t mazeCol = 0; mazeCol < mazeSize; mazeCol++) {
                mazeLine[mazeCol] =
                    autonomousMapping.getMazeSymbol(mazeRow, mazeCol);
            }
            oled.drawString(0, displayRow, mazeLine);
        }
        return;
    }

    char position[] = "Position: (0,0)";
    position[11] = '0' + row;
    position[13] = '0' + col;

    oled.drawString(0, 0, complete ? "Maze Complete!" : "Mapping Maze");
    oled.drawString(0, 2, position);

    const char* statusText = STATUS_READY_TEXT;
    switch (status) {
        case AutonomousMapping::STATUS_MOVING: statusText = STATUS_MOVING_TEXT; break;
        case AutonomousMapping::STATUS_TURNING: statusText = STATUS_TURNING_TEXT; break;
        case AutonomousMapping::STATUS_FRONT_WALL: statusText = STATUS_FRONT_WALL_TEXT; break;
        case AutonomousMapping::STATUS_SIDE_WALL: statusText = STATUS_SIDE_WALL_TEXT; break;
        case AutonomousMapping::STATUS_LIDAR_TIMEOUT: statusText = STATUS_TIMEOUT_TEXT; break;
        case AutonomousMapping::STATUS_NO_ROUTE: statusText = STATUS_NO_ROUTE_TEXT; break;
        case AutonomousMapping::STATUS_COMPLETE: statusText = STATUS_COMPLETE_TEXT; break;
        case AutonomousMapping::STATUS_READY:
        default: break;
    }
    char statusLine[14];
    strcpy_P(statusLine, statusText);
    oled.drawString(0, 4, statusLine);
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
        oled.drawString(0, 1, "Ready!!!");
    }

    delay(1000);

    // Side LiDARs only steer away when a wall is closer than 50 mm.
    robot.enableSideLidarAvoidance(true, 50.0);

    // Start the next turn if the front wall is closer than 50 mm.
    robot.enableFrontLidarSafety(true, 50);

    // robot.startCommandString(command_string, 130);
    // robot.startTurnHold(-90.0);
    // since its from the bottom, plus 2cm so its plus 20cm
    // robot.startWallDistance(120);   // DIRIVING AND STOPPING TASK 
    // robot.startStraightLine(3000, 130); // STRAIGHT LINE TRACKING TASK

    // 4.3
    autonomousMapping.begin(
        4,                              // start row
        1,                              // start column
        AutonomousMapping::SOUTH,       // start direction
        5,                              // goal row
        6                               // goal column
);

}

/*
void loop() {
    robot.update();
    delay(10);
} 
*/

void loop() {
    autonomousMapping.update();
    updateMappingOled();
    delay(10);
}
