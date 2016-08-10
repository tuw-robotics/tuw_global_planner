# tuw_path_planning
Various modules used for plath-planning [https://github.com/tuw-robotics/tuw_path_planning] 

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
