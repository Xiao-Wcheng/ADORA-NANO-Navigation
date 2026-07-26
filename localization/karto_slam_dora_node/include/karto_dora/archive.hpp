#pragma once

#include "karto_dora/types.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace karto_dora {

enum class ConstraintCategory { Sequential, NearChain, LoopClosure };
enum class SolverStatusArchive { NotRun, Converged, Failed };

struct LaserModelArchive {
  double range_min{0.0};
  double range_max{0.0};
  double angle_min{0.0};
  double angle_max{0.0};
  double angle_increment{0.0};
  double extrinsic_x{0.0};
  double extrinsic_y{0.0};
  double extrinsic_yaw{0.0};
};

struct ScanArchive {
  std::int64_t id{0};
  double stamp{0.0};
  Pose2d odometric_pose;
  Pose2d optimized_pose;
  std::vector<Point2d> points;
};

struct ConstraintArchive {
  std::int64_t source_id{0};
  std::int64_t target_id{0};
  Pose2d relative_pose;
  // Symmetric 3x3 upper triangle: xx, xy, xt, yy, yt, tt.
  std::array<double, 6> information{};
  bool loop_closure{false};
  ConstraintCategory category{ConstraintCategory::Sequential};
};

struct SolverMetadataArchive {
  std::size_t node_count{0};
  std::size_t constraint_count{0};
  std::size_t loop_closure_count{0};
  double initial_cost{0.0};
  double final_cost{0.0};
  bool converged{false};
  int iterations{0};
  std::size_t residual_blocks{0};
  SolverStatusArchive status{SolverStatusArchive::NotRun};
};

struct PoseGraphArchive {
  std::string source_tag;
  std::string config_hash;
  LaserModelArchive laser;
  std::vector<ScanArchive> scans;
  std::vector<ConstraintArchive> constraints;
  SolverMetadataArchive solver;
};

bool operator==(const LaserModelArchive &a, const LaserModelArchive &b);
bool operator==(const ScanArchive &a, const ScanArchive &b);
bool operator==(const ConstraintArchive &a, const ConstraintArchive &b);
bool operator==(const SolverMetadataArchive &a, const SolverMetadataArchive &b);
bool operator==(const PoseGraphArchive &a, const PoseGraphArchive &b);

void SaveArchiveAtomic(const std::filesystem::path &path,
                       const PoseGraphArchive &archive);
PoseGraphArchive LoadArchive(const std::filesystem::path &path);

}  // namespace karto_dora
