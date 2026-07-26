#pragma once

#include "karto_dora/config.hpp"
#include "karto_dora/archive.hpp"
#include "karto_dora/ceres_pose_graph_solver.hpp"
#include "karto_dora/scan_preparer.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace karto_dora {

struct ProcessResult {
  bool accepted{false};
  bool skipped{false};
  bool keyframe{false};
  bool loop_closed{false};
  Pose2d corrected_pose;
  double match_response{0.0};
  double covariance_xx{0.0};
  double covariance_yy{0.0};
  double covariance_yaw_yaw{0.0};
  std::size_t constraint_count{0};
};

enum class GraphConstraintKind { Sequential, NearChain, LoopClosure };

struct GraphNodeSnapshot {
  std::int64_t id{0};
  double stamp{0.0};
  Pose2d odometric_pose;
  Pose2d optimized_pose;
  std::vector<Point2d> points;
};

struct GraphConstraintSnapshot {
  std::int64_t source_id{0};
  std::int64_t target_id{0};
  Pose2d relative_pose;
  std::array<double, 6> information{};
  GraphConstraintKind kind{GraphConstraintKind::Sequential};
};

struct GraphSnapshot {
  std::vector<GraphNodeSnapshot> nodes;
  std::vector<GraphConstraintSnapshot> constraints;
  std::size_t sequential_constraints{0};
  std::size_t near_chain_constraints{0};
  std::size_t loop_closures{0};
  std::size_t loop_candidate_checks{0};
  std::size_t loop_fine_checks{0};
  std::size_t loop_rejections{0};
  OptimizationReport solver;
};

class KartoMapper {
 public:
  explicit KartoMapper(const SlamConfig &config);
  ~KartoMapper();

  KartoMapper(const KartoMapper &) = delete;
  KartoMapper &operator=(const KartoMapper &) = delete;

  ProcessResult Process(const PreparedScan &scan);
  ProcessResult Localize(const PreparedScan &scan);
  void SetInitialPose(const Pose2d &pose);
  void Restore(const PoseGraphArchive &archive);
  std::size_t scan_count() const;
  std::size_t graph_node_count() const;
  GraphSnapshot Snapshot() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace karto_dora
