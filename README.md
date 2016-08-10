# tuw_path_planning
Various modules used for plath-planning [https://github.com/tuw-robotics/tuw_path_planning] 

## tuw_waypoint_to_spline
converts a point array with chassis orientation into a spline functions with knots and control points
### parameters
- global_frame: [optional] frame id which is only used if the path was generated using a input file
- path_file: [optional] yaml input file with three vectors named x, y and o. x and y are desribing the position and o the relative chassis orientation to the path 
- waypoints_distance: [default: 0.5 ] discretization distance to generate points on the path for spline generation
- path_tmp_file: [default: /tmp/waypoint_to_spline.yaml ] if given a tmp file will be gernerated to store the points used to create the spline
