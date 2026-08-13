# MTRN3100 Micromouse

Arduino Nano starter code for the MTRN3100 micromouse project. The current code is organised around Task 3 simple driving: straight-line tracking, wall-distance stopping, turning, and chained maze commands.

The repository also contains a computer-side path-planning core for Task 4.2.

## Project Files

- `src/main.cpp`: Arduino entry point. Selects which task to run and creates the motors, encoders, lidar, IMU, and robot object.
- `include/Robot.hpp`: High-level robot behaviour for Task 3. Combines sensors, motors, and PID controllers into movement modes.
- `include/Motor.hpp`: Low-level motor PWM and direction control.
- `include/Encoder.hpp`: Encoder counting and wheel rotation measurement.
- `include/PIDController.hpp`: PID control helper.
- `include/Lidar.hpp`: Front distance sensor interface.
- `include/IMU.hpp`: Heading/yaw sensor interface.
- `computer/task4/planner.py`: Python occupancy-grid planner for Task 4.2. It
  performs obstacle inflation, eight-connected A* search, safe line-of-sight
  simplification, and waypoint/motion-command generation.
- `mazemappingtask2/maze_mapping_task2.ipynb`: Notebook workflow for the full
  Task 4.2 two-map route: discrete normal maze, separate continuous 5 x 5
  obstacle map, then the remaining discrete normal maze.
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

## Selecting A Task

In `src/main.cpp`, change `ACTIVE_TASK`:

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

- motor pin setup and PWM output in `include/Motor.hpp`
- encoder interrupts, counting, and rotation conversion in `include/Encoder.hpp`
- PID maths in `include/PIDController.hpp`
- real lidar library calls in `include/Lidar.hpp`
- real IMU library calls and gyro calibration in `include/IMU.hpp`
- command-string state machine in `include/Robot.hpp`

Also update the motor and encoder pin numbers in `src/main.cpp` to match your robot wiring.

## Task 3 Notes

- For straight-line tracking, use the IMU to correct heading while encoders measure distance.
- For wall stopping, use the front lidar and tune the distance PID to settle at 100 mm from the wall.
- For turning, use the IMU yaw angle and tune the turn PID until it stops within the required tolerance.
- For chained movement, reuse the same forward and turn behaviours for each command.
