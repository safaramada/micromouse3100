# MTRN3100 Micromouse

Arduino Nano starter code for the MTRN3100 micromouse project. The current code is organised around Task 3 simple driving: straight-line tracking, wall-distance stopping, turning, and chained maze commands.

## Project Files

- `main.ino`: Arduino entry point. Selects which task to run and creates the motors, encoders, lidar, IMU, and robot object.
- `Robot.hpp`: High-level robot behaviour for Task 3. Combines sensors, motors, and PID controllers into movement modes.
- `Motor.hpp`: Low-level motor PWM and direction control.
- `Encoder.hpp`: Encoder counting and wheel rotation measurement.
- `PIDController.hpp`: PID control helper.
- `Lidar.hpp`: Front distance sensor interface.
- `IMU.hpp`: Heading/yaw sensor interface.

## Selecting A Task

In `main.ino`, change `ACTIVE_TASK`:

```cpp
#define ACTIVE_TASK RUN_STRAIGHT_LINE
```

Available modes:

- `RUN_IDLE`: robot stays stopped
- `RUN_STRAIGHT_LINE`: drive 1 m forward
- `RUN_WALL_DISTANCE`: hold 100 mm from a front wall
- `RUN_TURN`: turn 90 degrees clockwise and hold heading
- `RUN_COMMAND_STRING`: run a command string such as `lfrfflfr`

## What Still Needs Implementation

The current files are starter code. Before the robot can run properly, fill in:

- motor pin setup and PWM output in `Motor.hpp`
- encoder interrupts, counting, and rotation conversion in `Encoder.hpp`
- PID maths in `PIDController.hpp`
- real lidar library calls in `Lidar.hpp`
- real IMU library calls and gyro calibration in `IMU.hpp`
- command-string state machine in `Robot.hpp`

Also update the motor and encoder pin numbers in `main.ino` to match your robot wiring.

## Task 3 Notes

- For straight-line tracking, use the IMU to correct heading while encoders measure distance.
- For wall stopping, use the front lidar and tune the distance PID to settle at 100 mm from the wall.
- For turning, use the IMU yaw angle and tune the turn PID until it stops within the required tolerance.
- For chained movement, reuse the same forward and turn behaviours for each command.
