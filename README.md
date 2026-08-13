# MTRN3100 Micromouse

Arduino Nano code for the MTRN3100 micromouse project. The current firmware is
configured for Task 4.3 online autonomous mapping. The straight-driving and
turning controllers required by the mapper remain inside `Robot.hpp`.

The repository also contains a computer-side path-planning core for Task 4.2.

## Project Files

- `src/main.cpp`: Arduino entry point. Creates the hardware objects and starts Task 4.3.
- `include/Robot.hpp`: Task 4.3 motion layer. Provides the proven straight-line
  and turning controls used by autonomous mapping.
- `include/Motor.hpp`: Low-level motor PWM and direction control.
- `include/Encoder.hpp`: Encoder counting and wheel rotation measurement.
- `include/PIDController.hpp`: PID control helper.
- `include/Lidar.hpp`: Front distance sensor interface.
- `include/IMU.hpp`: Heading/yaw sensor interface.
- `include/AutonomousMapping.hpp`: Non-blocking 9 x 9 online maze exploration.
  It senses walls, explores unvisited cells using depth-first search, backtracks
  through parent cells, and stops safely at the configured goal or on an error.
- `computer/task4/planner.py`: Python occupancy-grid planner for Task 4.2. It
  performs obstacle inflation, eight-connected A* search, safe line-of-sight
  simplification, and waypoint/motion-command generation.
- `test/test_task4_planner.py`: Unit tests for the Task 4.2 planner.

## Task 4.2 Planner

The camera/CV program should threshold and calibrate its image into a 2-D binary
mask (`0` free, non-zero occupied). Keep the outer course walls in that mask;
pixels outside the supplied map are treated as occupied. For reasonable Python
performance, resize the calibrated course to roughly 5--10 mm per pixel before
planning.

```python
from computer.task4 import ContinuousPlanner, OccupancyGrid, PlannerConfig

# obstacle_mask can be a 2-D NumPy/OpenCV array.
grid = OccupancyGrid.from_rows(obstacle_mask, resolution_mm=5.0)
planner = ContinuousPlanner(
    PlannerConfig(
        robot_radius_mm=50.0,  # replace with measured footprint radius
        safety_margin_mm=10.0,
    )
)
path = planner.plan(grid, start=(start_x, start_y), goal=(goal_x, goal_y))

if path.succeeded:
    # Draw path.grid_waypoints over the image for the demonstrator.
    commands = planner.make_motion_commands(
        path.waypoints_mm, initial_heading_deg=measured_start_heading_deg
    )
    # Send commands, or path.relative_waypoints_mm, over serial.
```

The planner belongs on the computer because the occupancy image and A* working
memory are much larger than an Arduino Nano should handle. Only the resulting
small waypoint/command list should be sent to the robot. Keep future OpenCV and
serial-link code beside the planner under `computer/` so PlatformIO does not try
to compile it for the Nano.

Run the planner tests on a computer with:

```sh
python3 -m unittest test/test_task4_planner.py
```

## Active Task 4.3 Firmware

`src/main.cpp` currently starts the mapper with:

```cpp
autonomousMapping.begin(0, 0, AutonomousMapping::SOUTH, 4, 7, true);
```

The final argument controls completion:

- `true`: stop when the goal is first reached.
- `false`: continue until every cell reachable from the start has been mapped.

Serial logging is disabled in the autonomous firmware so UART formatting and
transmission cannot disturb the control-loop timing. The OLED only reports
startup readiness.

The main physical tuning constants are near the top of
`include/AutonomousMapping.hpp`: `wallThresholdMm`, `forwardPwm`, and the
180 mm cell movement passed to `startStraightLine()`.

## Hardware Checks Still Required

The software builds for the Nano, but these values must match the physical
robot before an unattended maze run:

- Motor, encoder, XSHUT, and I2C wiring in `src/main.cpp`.
- Encoder counts per revolution and wheel radius.
- The measured centre-to-centre maze-cell distance.
- LiDAR wall threshold and sensor mounting offsets.
- Turn sign and PID tuning on the real drivetrain.
