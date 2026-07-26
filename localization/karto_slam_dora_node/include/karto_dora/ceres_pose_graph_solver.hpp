#pragma once

#include "karto_dora/types.hpp"

#include <karto_sdk/Mapper.h>
#include <Eigen/Core>

#include <optional>
#include <unordered_map>
#include <vector>

namespace karto_dora {

struct ConstraintNoise {
  double x_sigma{0.05};
  double y_sigma{0.05};
  double yaw_sigma{0.05};
};

enum class SolverStatus {
  NotRun,
  Converged,
  Failed
};

struct OptimizationReport {
  SolverStatus status{SolverStatus::NotRun};
  bool usable{false};
  int iterations{0};
  std::size_t residual_blocks{0};
  double initial_cost{0.0};
  double final_cost{0.0};
  std::string summary;
  std::unordered_map<int, Pose2d> final_poses;
};

class CeresPoseGraphSolver final : public karto::ScanSolver {
public:
  CeresPoseGraphSolver();
  ~CeresPoseGraphSolver() override = default;

  void Configure() override;
  void Compute() override;
  const IdPoseVector &GetCorrections() const override;
  void AddNode(karto::Vertex<karto::LocalizedRangeScan> *vertex) override;
  void AddConstraint(karto::Edge<karto::LocalizedRangeScan> *edge) override;
  void RemoveNode(kt_int32s id) override;
  void RemoveConstraint(kt_int32s source_id, kt_int32s target_id) override;
  void Clear() override;

  void AddPoseNode(int id, const Pose2d &pose);
  void AddPoseConstraint(int source, int target, const Pose2d &relative,
                         const ConstraintNoise &noise);
  OptimizationReport ComputeReport();
  const OptimizationReport &LatestReport() const noexcept;
  std::optional<Pose2d> PoseFor(int id) const;

private:
  struct Constraint {
    int source;
    int target;
    Pose2d relative;
    Eigen::Matrix3d sqrt_information;
  };

  std::unordered_map<int, Eigen::Vector3d> nodes_;
  std::vector<Constraint> constraints_;
  IdPoseVector corrections_;
  OptimizationReport last_report_;
};

}  // namespace karto_dora
