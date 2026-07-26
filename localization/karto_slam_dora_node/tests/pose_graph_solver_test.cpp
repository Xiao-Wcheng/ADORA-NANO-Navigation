#include "karto_dora/ceres_pose_graph_solver.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
  karto_dora::CeresPoseGraphSolver empty_solver;
  assert(empty_solver.LatestReport().status == karto_dora::SolverStatus::NotRun);

  empty_solver.AddPoseNode(0, {0.0, 0.0, 0.0});
  const auto failed = empty_solver.ComputeReport();
  assert(failed.status == karto_dora::SolverStatus::Failed);
  assert(failed.residual_blocks == 0);

  karto_dora::CeresPoseGraphSolver solver;
  solver.AddPoseNode(0, {0.0, 0.0, 0.0});
  solver.AddPoseNode(1, {1.05, 0.0, 0.02});
  solver.AddPoseNode(2, {1.08, 1.06, 1.59});
  solver.AddPoseNode(3, {0.02, 1.10, 3.13});
  solver.AddPoseNode(4, {-0.18, 0.12, -1.50});

  const karto_dora::ConstraintNoise odom{0.03, 0.03, 0.03};
  const karto_dora::ConstraintNoise loop{0.01, 0.01, 0.01};
  solver.AddPoseConstraint(0, 1, {1.0, 0.0, 0.0}, odom);
  solver.AddPoseConstraint(1, 2, {0.0, 1.0, 1.5707963267948966}, odom);
  solver.AddPoseConstraint(2, 3, {0.0, 1.0, 1.5707963267948966}, odom);
  solver.AddPoseConstraint(3, 4, {0.0, 1.0, 1.5707963267948966}, odom);
  solver.AddPoseConstraint(4, 0, {0.0, 0.0, 1.5707963267948966}, loop);

  const auto report = solver.ComputeReport();
  assert(report.usable);
  assert(report.status == karto_dora::SolverStatus::Converged);
  assert(report.residual_blocks == 5);
  assert(report.final_poses.size() == 5);
  assert(solver.LatestReport().status == karto_dora::SolverStatus::Converged);
  const auto pose4 = solver.PoseFor(4);
  assert(pose4.has_value());
  const double position_error = std::hypot(pose4->x, pose4->y);
  assert(position_error < 0.08);
  assert(report.final_cost < report.initial_cost);

  // Karto localization keeps only a bounded scan buffer. Removed scans must
  // also disappear from the optimizer, otherwise later corrections reference
  // deleted scan IDs and can corrupt the localization pose.
  solver.RemoveConstraint(3, 4);
  solver.RemoveConstraint(4, 0);
  solver.RemoveNode(4);
  assert(!solver.PoseFor(4).has_value());
  const auto pruned_report = solver.ComputeReport();
  assert(pruned_report.usable);
  assert(pruned_report.final_poses.count(4) == 0);
  assert(pruned_report.residual_blocks == 3);

  solver.Clear();
  assert(solver.LatestReport().status == karto_dora::SolverStatus::Converged);
  assert(!solver.PoseFor(4).has_value());
  std::cout << "pose_graph_solver_test PASS error=" << position_error << "\n";
}
