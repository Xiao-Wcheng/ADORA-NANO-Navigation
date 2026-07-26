// Ceres graph formulation derived from slam_toolbox 2.6.9
// solvers/ceres_solver.cpp and solvers/ceres_utils.h.
#include "karto_dora/ceres_pose_graph_solver.hpp"

#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>
#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

namespace karto_dora {
namespace {

template<typename T>
T Normalize(const T &angle)
{
  const T two_pi(2.0 * M_PI);
  return angle - two_pi * ceres::floor((angle + T(M_PI)) / two_pi);
}

class AngleParameterization {
public:
  template<typename T>
  bool operator()(const T *theta, const T *delta, T *result) const
  {
    *result = Normalize(*theta + *delta);
    return true;
  }
};

template<typename T>
Eigen::Matrix<T, 2, 2> Rotation(T yaw)
{
  Eigen::Matrix<T, 2, 2> rotation;
  rotation << ceres::cos(yaw), -ceres::sin(yaw), ceres::sin(yaw), ceres::cos(yaw);
  return rotation;
}

class PoseGraphError {
public:
  PoseGraphError(const Pose2d &relative, const Eigen::Matrix3d &sqrt_information)
    : translation_(relative.x, relative.y), yaw_(relative.yaw),
      sqrt_information_(sqrt_information) {}

  template<typename T>
  bool operator()(const T *xa, const T *ya, const T *yawa,
                  const T *xb, const T *yb, const T *yawb,
                  T *residuals) const
  {
    const Eigen::Matrix<T, 2, 1> a(*xa, *ya);
    const Eigen::Matrix<T, 2, 1> b(*xb, *yb);
    Eigen::Map<Eigen::Matrix<T, 3, 1>> r(residuals);
    r.template head<2>() = Rotation(*yawa).transpose() * (b - a) - translation_.cast<T>();
    r(2) = Normalize((*yawb - *yawa) - T(yaw_));
    r = sqrt_information_.cast<T>() * r;
    return true;
  }

  static ceres::CostFunction *Create(const Pose2d &relative,
                                     const Eigen::Matrix3d &sqrt_information)
  {
    return new ceres::AutoDiffCostFunction<PoseGraphError, 3, 1, 1, 1, 1, 1, 1>(
      new PoseGraphError(relative, sqrt_information));
  }

private:
  Eigen::Vector2d translation_;
  double yaw_;
  Eigen::Matrix3d sqrt_information_;
};

Eigen::Matrix3d InformationFromNoise(const ConstraintNoise &noise)
{
  if (noise.x_sigma <= 0.0 || noise.y_sigma <= 0.0 || noise.yaw_sigma <= 0.0) {
    throw std::invalid_argument("constraint sigma must be positive");
  }
  Eigen::Matrix3d result = Eigen::Matrix3d::Zero();
  result(0, 0) = 1.0 / noise.x_sigma;
  result(1, 1) = 1.0 / noise.y_sigma;
  result(2, 2) = 1.0 / noise.yaw_sigma;
  return result;
}

}  // namespace

CeresPoseGraphSolver::CeresPoseGraphSolver() = default;

void CeresPoseGraphSolver::Configure() {}

void CeresPoseGraphSolver::Compute()
{
  ComputeReport();
}

const karto::ScanSolver::IdPoseVector &CeresPoseGraphSolver::GetCorrections() const
{
  return corrections_;
}

void CeresPoseGraphSolver::AddPoseNode(int id, const Pose2d &pose)
{
  nodes_[id] = Eigen::Vector3d(pose.x, pose.y, pose.yaw);
}

void CeresPoseGraphSolver::AddPoseConstraint(int source, int target,
                                              const Pose2d &relative,
                                              const ConstraintNoise &noise)
{
  constraints_.push_back({source, target, relative, InformationFromNoise(noise)});
}

void CeresPoseGraphSolver::AddNode(karto::Vertex<karto::LocalizedRangeScan> *vertex)
{
  if (!vertex || !vertex->GetObject()) return;
  const karto::Pose2 pose = vertex->GetObject()->GetCorrectedPose();
  AddPoseNode(vertex->GetObject()->GetUniqueId(), {pose.GetX(), pose.GetY(), pose.GetHeading()});
}

void CeresPoseGraphSolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan> *edge)
{
  if (!edge || !edge->GetSource() || !edge->GetTarget()) return;
  auto *link = static_cast<karto::LinkInfo *>(edge->GetLabel());
  if (!link) return;
  const int source = edge->GetSource()->GetObject()->GetUniqueId();
  const int target = edge->GetTarget()->GetObject()->GetUniqueId();
  const karto::Pose2 pose = link->GetPoseDifference();
  const karto::Matrix3 precision = link->GetCovariance().Inverse();
  Eigen::Matrix3d information;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) information(row, col) = precision(row, col);
  }
  Eigen::LLT<Eigen::Matrix3d> llt(information);
  if (llt.info() != Eigen::Success) throw std::runtime_error("constraint covariance is not positive definite");
  constraints_.push_back({source, target, {pose.GetX(), pose.GetY(), pose.GetHeading()},
                          llt.matrixU()});
}

void CeresPoseGraphSolver::RemoveNode(kt_int32s id)
{
  nodes_.erase(id);
  constraints_.erase(
    std::remove_if(
      constraints_.begin(), constraints_.end(),
      [id](const Constraint & constraint) {
        return constraint.source == id || constraint.target == id;
      }),
    constraints_.end());
  corrections_.erase(
    std::remove_if(
      corrections_.begin(), corrections_.end(),
      [id](const auto & correction) { return correction.first == id; }),
    corrections_.end());
}

void CeresPoseGraphSolver::RemoveConstraint(
  kt_int32s source_id, kt_int32s target_id)
{
  constraints_.erase(
    std::remove_if(
      constraints_.begin(), constraints_.end(),
      [source_id, target_id](const Constraint & constraint) {
        return
          (constraint.source == source_id && constraint.target == target_id) ||
          (constraint.source == target_id && constraint.target == source_id);
      }),
    constraints_.end());
}

OptimizationReport CeresPoseGraphSolver::ComputeReport()
{
  OptimizationReport report;
  if (nodes_.empty()) {
    report.summary = "no pose nodes";
    last_report_ = report;
    return report;
  }

  ceres::Problem::Options problem_options;
  problem_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem(problem_options);
  auto angle = std::unique_ptr<ceres::LocalParameterization>(
    new ceres::AutoDiffLocalParameterization<AngleParameterization, 1, 1>());
  auto loss = std::make_unique<ceres::HuberLoss>(0.7);

  for (const Constraint &constraint : constraints_) {
    auto source = nodes_.find(constraint.source);
    auto target = nodes_.find(constraint.target);
    if (source == nodes_.end() || target == nodes_.end() || source == target) continue;
    problem.AddResidualBlock(PoseGraphError::Create(constraint.relative, constraint.sqrt_information),
      loss.get(), &source->second(0), &source->second(1), &source->second(2),
      &target->second(0), &target->second(1), &target->second(2));
    ++report.residual_blocks;
  }
  if (report.residual_blocks == 0) {
    report.status = SolverStatus::Failed;
    report.summary = "no valid pose constraints";
    last_report_ = report;
    return report;
  }
  for (auto &[id, pose] : nodes_) {
    (void)id;
    if (problem.HasParameterBlock(&pose(2))) problem.SetParameterization(&pose(2), angle.get());
  }
  const auto first = std::min_element(nodes_.begin(), nodes_.end(),
    [](const auto &a, const auto &b) { return a.first < b.first; });
  if (first != nodes_.end()) {
    for (int index = 0; index < 3; ++index) {
      if (problem.HasParameterBlock(&first->second(index))) problem.SetParameterBlockConstant(&first->second(index));
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.function_tolerance = 1e-3;
  options.gradient_tolerance = 1e-6;
  options.parameter_tolerance = 1e-3;
  options.max_num_iterations = 100;
  options.num_threads = 4;
  options.use_nonmonotonic_steps = true;
  options.dynamic_sparsity = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  report.usable = summary.IsSolutionUsable();
  report.status = report.usable ? SolverStatus::Converged : SolverStatus::Failed;
  report.iterations = static_cast<int>(summary.iterations.size());
  report.initial_cost = summary.initial_cost;
  report.final_cost = summary.final_cost;
  report.summary = summary.BriefReport();

  corrections_.clear();
  if (report.usable) {
    std::vector<int> ids;
    ids.reserve(nodes_.size());
    for (const auto &[id, pose] : nodes_) { (void)pose; ids.push_back(id); }
    std::sort(ids.begin(), ids.end());
    for (int id : ids) {
      const auto &pose = nodes_.at(id);
      corrections_.emplace_back(id, karto::Pose2(pose(0), pose(1), pose(2)));
      report.final_poses.emplace(id, Pose2d{pose(0), pose(1), pose(2)});
    }
  }
  last_report_ = report;
  return report;
}

const OptimizationReport &CeresPoseGraphSolver::LatestReport() const noexcept
{
  return last_report_;
}

std::optional<Pose2d> CeresPoseGraphSolver::PoseFor(int id) const
{
  const auto found = nodes_.find(id);
  if (found == nodes_.end()) return std::nullopt;
  return Pose2d{found->second(0), found->second(1), found->second(2)};
}

void CeresPoseGraphSolver::Clear()
{
  nodes_.clear();
  constraints_.clear();
  corrections_.clear();
}

}  // namespace karto_dora
