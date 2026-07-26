# Navigation Interfaces

The app uses JSON payloads over Dora outputs. The field names are intentionally
close to ROS2 concepts, but there is no ROS runtime dependency.

## LaserScan

Producer: `ms200`

Main fields:

- `angle_min`
- `angle_max`
- `angle_increment`
- `range_min`
- `range_max`
- `ranges`
- `header.frame_id = lidar`

## Odometry

Producer: `chassis`

Main fields:

- `pose.x`
- `pose.y`
- `pose.theta`
- `localization_mode`
- `localization_matched`
- `localization_score`
- `localization_matches`
- `localization_lost_count`
- `twist.linear.x`
- `twist.linear.y`
- `twist.angular.z`

The current odometry is wheel-kinematics odometry from the Feetech kiwi chassis.

## CorrectedPose

Producers:

- `scan_matching`: local scan-matching corrected pose.
- `map_localization`: scan-to-map corrected pose in `--localize` mode.
- `pose_graph_slam`: global pose-graph corrected pose in `--slam` mode.

Main fields:

- `pose.x`
- `pose.y`
- `pose.theta`

This is the current pose input for mapping, goal conversion, path tracking, and
supervision.

## PoseGraph

Producer: `pose_graph_slam`

Main fields:

- `num_keyframes`
- `num_constraints`
- `loop_closures`
- `constraints`

This is diagnostic graph data for SLAM development.

Loop-closure constraints use scan-to-keyframe matching in `--slam` mode. The
main tuning parameters are:

- `LOOP_MATCH_SEARCH_XY`
- `LOOP_MATCH_SEARCH_YAW_DEG`
- `LOOP_MATCH_MAX_DIST`
- `LOOP_MATCH_SCORE_THRESHOLD`
- `LOOP_MATCH_MIN_POINTS`

Graph optimization uses lightweight SE(2) constraint-error iterations. The main
tuning parameters are:

- `SMOOTHING_ITERATIONS`
- `SMOOTHING_ALPHA`
- `MAX_POSE_UPDATE_XY`
- `MAX_POSE_UPDATE_YAW_DEG`

Mode parameters:

- `SLAM_MODE=mapping`: add keyframes, optimize graph, and save graph.
- `SLAM_MODE=localization`: load a saved graph and match scans against saved
  keyframes without changing the graph or map.
- `GRAPH_PATH`: JSON pose graph path.

## SlamTrajectory

Producer: `pose_graph_slam`

Main fields:

- `keyframes`
- optimized and raw pose for each keyframe

This is the optimized keyframe trajectory used for SLAM diagnostics.

## OccupancyGrid

Producer: `mapper`

The saved map is consumed by the A* planner through `MAP_YAML_PATH`.

Map convention:

- `0`: occupied in PGM
- `254`: free in PGM
- `205`: unknown in PGM

## Goal

Producer: `pose_goal`

Outputs:

- `start_point`: A* grid start point.
- `goal_point`: A* grid target point.
- `goal_pose`: world-frame goal for supervision.

Goal modes:

- relative: `GOAL_DISTANCE` and `GOAL_LATERAL`
- absolute: `GOAL_X` and `GOAL_Y`

## Path

Producer: `path_planning_node`

Consumer: `path_tracking`

Main fields:

- `path_found`
- `waypoints`

Planner map parameters:

- `INFLATION_RADIUS_CELLS`: expands occupied cells before planning.
- `RELOAD_MAP_ON_PLAN`: reloads the map file before each global planning
  request. This lets the planner consume maps saved by the mapper during a
  Dora run.
- `MAX_NEAREST_FREE_RADIUS_CELLS`: if the requested start or goal is inside an
  inflated obstacle, search nearby cells for a free replacement.

## CmdVelTwist

Producers:

- `path_tracking`: desired path-following velocity.
- `local_planner`: final safe velocity to chassis.

Main fields:

- `linear.x`: forward/backward
- `linear.y`: left/right
- `angular.z`: CCW/CW rotation

Direction convention:

- `linear.x > 0`: forward
- `linear.y > 0`: left
- `angular.z > 0`: counter-clockwise

## NavigationStatus

Producer: `nav_supervisor`

States:

- `RUNNING`
- `REACHED`
- `BLOCKED`
- `TIMEOUT`
- `POSE_TIMEOUT`
- `LOCALIZATION_LOST`

`LOCALIZATION_LOST` means pose messages are still arriving, but localization
matching has failed for too many consecutive updates. In `--localize` mode the
local planner also uses this condition to stop the chassis.
