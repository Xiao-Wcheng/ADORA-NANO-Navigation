#pragma once

#include <string>
#include <vector>

namespace karto_dora {

struct Pose2d {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct TimedPose2d {
  double stamp{0.0};
  Pose2d pose;
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

struct Point2d {
  double x{0.0};
  double y{0.0};
};

struct LaserScan {
  std::string frame_id;
  double start_stamp{0.0};
  double end_stamp{0.0};
  double angle_min{0.0};
  double angle_max{0.0};
  double angle_increment{0.0};
  double scan_time{0.0};
  double time_increment{0.0};
  double range_min{0.0};
  double range_max{0.0};
  std::vector<double> ranges;
  std::vector<double> intensities;
};

}  // namespace karto_dora
