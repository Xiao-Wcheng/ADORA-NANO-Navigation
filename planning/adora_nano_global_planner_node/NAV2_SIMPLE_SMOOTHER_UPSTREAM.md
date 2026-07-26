# Nav2 SimpleSmoother upstream

The standalone smoother in this directory is adapted from:

- Project: Navigation2
- Package: `nav2_smoother`
- Component: `nav2_smoother::SimpleSmoother`
- Upstream source:
  https://github.com/ros-navigation/navigation2/blob/main/nav2_smoother/src/simple_smoother.cpp
- Upstream copyright: Copyright (c) 2022, Samsung Research America
- License: Apache License 2.0

The Dora port retains the upstream weighted data/smooth update, convergence
tolerance, iteration limit, refinement passes, fixed endpoints, dense pose
count, and infeasible-update rollback. ROS lifecycle, pluginlib,
`nav_msgs::Path`, parameter-server, and costmap-subscriber wrappers were
replaced by C++17 types and this project's `MapLoader`.
