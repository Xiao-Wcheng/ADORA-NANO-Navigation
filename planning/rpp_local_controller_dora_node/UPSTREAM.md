# Upstream provenance

This ROS-free Dora controller ports the control equations and behavioral
structure of `nav2_regulated_pure_pursuit_controller` from the Navigation2
project:

- Repository: https://github.com/ros-navigation/navigation2
- Package: `nav2_regulated_pure_pursuit_controller`
- Retrieved/reference date: 2026-07-23
- License: Apache License 2.0

Ported concepts include velocity-scaled lookahead, interpolated carrot
selection, pure-pursuit curvature, curvature regulation, approach regulation,
rotate-to-path behavior, and projected-command collision checking.

ROS-dependent lifecycle, pluginlib, parameters, TF, ROS messages, and
Costmap2D integration are replaced by Dora inputs, plain C++ types, environment
configuration, and the project's rolling laser costmap.
