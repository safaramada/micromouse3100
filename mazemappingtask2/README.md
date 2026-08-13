# Maze Mapping Task 2 - Two Independent Maps

This folder implements the Task 4.2 route as three planning stages:

```text
normal 9 x 9 maze -> continuous 5 x 5 obstacle area -> normal 9 x 9 maze
```

It does **not** plan all three stages on one noisy pixel occupancy image.

## Map 1: normal maze

`run_two_map_demo()` calls the exact Task 1 entry point:

```python
from mazemapping.mask_maze import create_maze_masks
task1_masks = create_maze_masks(rectified_image)
```

The normal planner samples the expected horizontal and vertical cell
boundaries from those Task 1 masks and builds a discrete 9 x 9 cell graph.
The configured 5 x 5 obstacle interior is removed from this graph, except for
the two inside portal cells. The only permitted crossings are the programmed
entrance and exit.

The normal graph is solved with a heading-aware shortest-path search. Commands
are generated directly from its logical cell path:

- `f` means move forward exactly one 180 mm cell;
- `l` means turn left 90 degrees;
- `r` means turn right 90 degrees;
- a 180-degree turn is written as `rr`.

No arbitrary-angle movement is generated for the normal maze.

## Map 2: obstacle area

The configured 5 x 5 region is perspective-warped into its own 900 mm by
900 mm metric occupancy map at 5 mm/pixel. Its cylinder detections are derived
from Task 1's exact `03a_main_dark_mask.png` output, then filtered using the
known approximate 100 mm cylinder size so wall pieces and small floor marks do
not become obstacles.

The robot radius plus safety margin is applied only to this metric obstacle
map. An eight-connected A* search finds a collision-free route and line-of-
sight simplification converts it to arbitrary-angle waypoints.

The portal cells are programmed deliberately. This matches the staff advice
that the 5 x 5 entrance and exit positions may be programmed, while the walls
and cylinder positions still come from the camera image.

## Joined result

The final command has this exact structure:

```cpp
"normal_lfr,[(turn_degrees,distance_mm),...],normal_lfr";
```

Only the middle bracketed section contains arbitrary angles and metric
distances. A zero-distance final tuple may be added there to align the robot
with the first cardinal direction of the second normal-maze section.

The continuous crop coordinates are transformed back into the full rectified
map, then the complete joined route is projected onto the original camera
image.

## Run and configure

Open and run all cells in `maze_mapping_task2.ipynb`. The checked-in example
uses `mazemappingtask2/maze.png`.

Change the input image in the notebook's **Configuration** cell:

```python
image_file=TASK2_DIR / 'maze.png'
```

That same cell contains the fixed-camera grid calibration, 5 x 5 location,
programmed entrance/exit, start/goal cells and optional goal heading, and the
measured robot radius. Full-maze coordinates use `(row, column)` from the top-left. Motion
headings are Cartesian: east `0`, north `90`, west `180`, south `-90`.

For a different fixed camera, calibrate `board_corners` and `grid_bounds` once
before assessment. Measure `robot_radius_mm` from the robot centre to its
furthest corner rather than retaining the example value.

## Main files

- `maze_mapping_task2.ipynb` - configuration, execution, and visual checks.
- `two_map_demo.py` - calls Task 1 masking and assembles all outputs.
- `two_map_planner.py` - normal graph, obstacle crop, both planners, commands,
  coordinate transforms, and drawing.
- `task2_pipeline.py` - older shared rectification/projection utilities; its
  previous one-map hybrid planner is not used by the new notebook.

Generative AI disclosure: OpenAI Codex assisted with the implementation and
tests. AI-assisted source sections are identified with inline comments.
