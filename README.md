# MTRN3100 Micromouse

Firmware for an Arduino Nano-based micromouse. The program supports straight-line driving, stopping at a wall, controlled turns, and command strings made from `f`, `l`, and `r`.

## Project structure

Configuration and interfaces live in `include/`; implementations live in `src/`.

| File | Responsibility |
| --- | --- |
| `include/Config.hpp` | Pin assignments, sensor addresses, and physical constants |
| `src/main.cpp` | Constructs the hardware and contains Arduino `setup()`/`loop()` |
| `Display.hpp` / `Display.cpp` | OLED setup, status messages, and I2C diagnostics |
| `Motor.hpp` / `Motor.cpp` | Motor direction and PWM output |
| `Encoder.hpp` / `Encoder.cpp` | Encoder interrupts, counts, and wheel rotation |
| `IMU.hpp` / `IMU.cpp` | MPU6050 setup, calibration, and integrated yaw |
| `Lidar.hpp` / `Lidar.cpp` | VL6180X startup, addressing, and distance readings |
| `Robot.hpp` / `Robot.cpp` | Robot startup and individual movement controllers |
| `RobotCommands.cpp` | Multi-step command parsing and wall-centering logic |

The code uses statically allocated objects and does not use `String`, dynamic allocation, or heap-backed containers. Serial debug text uses Arduino's `F()` macro so it remains in flash instead of consuming the Nano's limited SRAM.

## Hardware configuration

Update `include/Config.hpp` when wiring, encoder resolution, wheel size, or sensor addresses change. Encoder channel A pins must support external interrupts; the current Nano configuration uses pins 2 and 3.

The three VL6180X sensors start at the same default I2C address. `Robot::beginSensors()` holds all three in shutdown and then enables them one at a time so each can receive its configured address.

## Selecting behavior

`setup()` leaves the robot idle by default. Uncomment one behavior in `src/main.cpp` while testing:

```cpp
robot.startStraightLine(1000.0f);
robot.startWallDistance(120.0f);
robot.startTurnHold(-90.0f);
robot.startCommandString("frfrfflflff");
```

Optional command-mode aids can be enabled before starting the command string:

```cpp
robot.enableLidarCentering(true);
robot.enableFrontLidarSafety(true, 40);
```

Direction signs depend on the physical motor and IMU orientation. If a turn moves away from its target, verify the motor wiring and the sign of the requested angle.

## Build

Install PlatformIO, then run:

```sh
pio run
```

The configured environment is `nanoatmega328` in `platformio.ini`.
