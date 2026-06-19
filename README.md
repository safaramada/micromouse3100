# MTRN3100 Micromouse

PlatformIO firmware for the MTRN3100 group assignment.

## Week 4 barebones movement

On power-up, the robot waits 1.5 seconds and then performs the full sequence
recorded in the Week 4 starter code:

1. Drive forward approximately 200 mm and stop.
2. Make four 90-degree counter-clockwise turns.
3. Make four 90-degree clockwise turns and stop.

The sequence only runs once; resetting or power-cycling the Arduino starts a
new run.

The controller-board pin mapping used in `src/main.cpp` is:

| Function | Nano pin |
| --- | ---: |
| Left motor PWM | D11 |
| Left motor direction | D12 |
| Right motor PWM | D9 |
| Right motor direction | D10 |

### Before the assessed run

1. Put the robot on the same floor and use the same charged battery intended
   for marking.
2. Mark a start line, upload the program, and measure the distance travelled.
3. Update `DISTANCE_CALIBRATION_MS_PER_MM` using:

   `new value = old value * 200 / measured distance in mm`

   Example: if the current value is `5.0` and the robot travels 175 mm, use
   `5.0 * 200 / 175 = 5.71`.
4. Repeat until the robot's whole footprint stops between the 125 mm and
   275 mm goal lines.
5. If it curves, reduce the trim on the faster side (or increase the slower
   side) using `LEFT_DRIVE_TRIM` and `RIGHT_DRIVE_TRIM`.
6. Test one turn in each direction and adjust `LEFT_90_TIME_MS` and
   `RIGHT_90_TIME_MS` using:

   `new time = old time * 90 / measured turn angle in degrees`

   Repeat the four turns after calibration; the robot should finish facing its
   original direction after each set.

If forward commands make a wheel run backward, toggle that wheel's
`*_MOTOR_REVERSED` flag. Distance and turn timing are open-loop estimates, so
they must be checked on the actual robot before marking.
