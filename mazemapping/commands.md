**standard**
py mask_maze.py "maze.png" --show
py path_planning_nodes_fixed_grid.py "maze.png" --rows 9 --columns 9
py shortest_path_dijkstra.py "maze.png" --start 1 --goal 79
py path_to_lfr.py "maze.png" --start 1 --goal 79 --heading up

**cylinder**

py mask_maze_cylinders.py "cylinder_maze.png" --cylinder-buffer 10 --show
py path_planning_nodes_cylinders.py "cylinder_maze.png" --rows 9 --columns 9 --cylinder-buffer 10 --node-collision-radius 3 --show-mask
py shortest_path_cylinders.py "cylinder_maze.png" --start 28 --goal 70 --edge-thickness 1 --node-collision-radius 1