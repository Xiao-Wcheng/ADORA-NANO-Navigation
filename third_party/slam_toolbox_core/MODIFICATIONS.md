# Local Modifications

## 2026-07-21: Remove ROS configuration type from the algorithm interface

`karto_sdk/Mapper.h` upstream includes `rclcpp/rclcpp.hpp` only so the abstract
`ScanSolver::Configure` method can accept an ROS node. The Karto scan matching,
graph construction, loop detection, and grid algorithms do not otherwise use
ROS.

The Dora port removes that include and changes the abstract signature from
`Configure(rclcpp::Node::SharedPtr)` to `Configure()`. The standalone Ceres
solver receives its typed configuration through its constructor before this
method is called. No scan-matching or graph algorithm is changed.
