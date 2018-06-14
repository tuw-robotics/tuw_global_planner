# tuw_global_planner
This module holds modified clone of the global planner used in ros navigation of https://github.com/ros-planning/navigation.git 
## global_planner
This pkg implements a voronoi planner to route a single vehicle along a generated voronoi graph.  The voronoi graph is generated based on the received map message. 
Thanks goes to the ROS move_base comunity because this pkg holds a copy of the global_planner used in move_base https://github.com/ros-planning/navigation.git and extends it with a A-Star Planner with different Heuristics and a embedded Voronoi Graph Generator to replace the map with it.
### parameters
- __allow_unknown (bool, default: true)__:   Specifies whether or not to allow the planner to create plans that traverse unknown space. NOTE: if you are using a layered costmap_2d costmap with a voxel or obstacle layer, you must also set the track_unknown_space param for that layer to be true, or it will convert all your unknown space to free space (which planner will then happily go right through). 
- __default_tolerance (double, default: 0.0)__:   A tolerance on the goal point for the planner. The planner will attempt to create a plan that is as close to the specified goal as possible but no further than default_tolerance away. 
- __visualize_potential (bool, default: false)__:   Specifies whether or not to visualize the potential area computed via a PointCloud2. 
- __use_dijkstra (bool, default: true)__:   If true, use dijkstra's algorithm. Otherwise, A*. 
- __use_quadratic (bool, default: true)__:   If true, use the quadratic approximation of the potential. Otherwise, use a simpler calculation. 
- __use_grid_path (bool, default: false)__:   If true, create a path that follows the grid boundaries. Otherwise, use a gradient descent method. 
- __old_navfn_behavior (bool, default: false)__:   If for some reason, you want global_planner to exactly mirror the behavior of navfn, set this to true (and use the defaults for the other boolean parameters) 
- __use_astar_tuw__:  If true the TUW Astar mode is activated and a heuristic can be selected.
- __use_voronoi__:  If true the given map is converted to a voronoi map and the TUW AStar algorithm is used to plan a path
- __astar_heuristics__:  0 (no heuristic==dirjkstra), 1 (Manhatten), 2 (Diagonal), 3 (Euclidean distance)
- __heuristics_weight__:  With this parameter the weight of the heuristic can be adjusted (0 means dijkstra ... 1 means full heuristic)

## tuw_waypoint_to_spline
converts a point array with chassis orientation into a spline functions with knots and control points
### parameters
- __global_frame__: [optional] frame id which is only used if the path was generated using a input file
- __path_file__: [optional] yaml input file with three vectors named x, y and o. x and y are desribing the position and o the relative chassis orientation to the path 
- __waypoints_distance__: [default: 0.5 ] discretization distance to generate points on the path for spline generation
- __path_tmp_file__: [default: /tmp/waypoint_to_spline.yaml ] if given a tmp file will be gernerated to store the points used to create the spline

### arguments examples
```
_global_frame:=map _waypoints_distance:=0.5 path:=/global_planner/planner/plan __ns:=blue
```
