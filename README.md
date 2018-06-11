# tuw_path_planning
This module holds modified clone of the global planner used in ros navigation of https://github.com/ros-planning/navigation.git 
## tuw_path_planning
This is just the metapackage
## global_planner
This pkg implements a voronoi planner to route a single vehicle along a generated voronoi graph. 
    The voronoi graph is generated based on the received map message. 
    Thanks goes to the ROS move_base comunity because this pkg holds a copy of the global_planner used in move_base https://github.com/ros-planning/navigation.git. 
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
