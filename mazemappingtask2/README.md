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
map. A deterministic RRT* search finds a collision-free route. Its edges are
collision-checked, redundant internal tree nodes are removed, and the route is
resampled at a configurable maximum spacing (150 mm by default). This retains
useful checkpoints instead of issuing one long drive through the area.

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

The continuous drive distances use the measured robot calibration
`1000 / 1018.2 = 0.9821`. This scale changes only positive distances inside
the brackets. It does not change turn angles, zero-distance alignment, or the
normal maze's 180 mm `f` commands.

The continuous crop coordinates are transformed back into the full rectified
map, then the complete joined route is projected onto the original camera
image.

## Run and configure

Open and run all cells in `maze_mapping_task2.ipynb`. The notebook is split
into two phases so a changed entrance or exit cannot prevent coordinate setup:

1. choose the image and run the labelled 9 x 9 grid preview;
2. manually enter the obstacle-region location, start/goal, and the inside and
   outside cell on each portal before running the planner.

Change the input image in the notebook's **Phase 1** cell:

```python
IMAGE_FILE=TASK2_DIR / 'maze2.png'
```

Phase 1 also contains the fixed-camera grid calibration. Phase 2 contains the
5 x 5 location, programmed entrance/exit, start/goal cells, optional goal
heading, and measured robot radius. Full-maze coordinates use `(row, column)`
from the top-left. Motion
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
