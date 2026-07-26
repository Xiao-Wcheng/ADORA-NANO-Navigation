#include "karto_dora/runtime.hpp"

#include "karto_dora/message_adapter.hpp"
#include "karto_dora/occupancy_grid.hpp"
#include "karto_dora/scan_preparer.hpp"

#include <nlohmann/json.hpp>

#include <stdexcept>

namespace karto_dora {
namespace {

PoseGraphArchive ArchiveFromSnapshot(const GraphSnapshot &snapshot,
                                     const PoseGraphArchive &header)
{
  PoseGraphArchive archive;
  archive.source_tag = header.source_tag;
  archive.config_hash = header.config_hash;
  archive.laser = header.laser;
  archive.scans.reserve(snapshot.nodes.size());
  for (const auto &node : snapshot.nodes) {
    archive.scans.push_back({node.id, node.stamp, node.odometric_pose,
                             node.optimized_pose, node.points});
  }
  archive.constraints.reserve(snapshot.constraints.size());
  for (const auto &edge : snapshot.constraints) {
    ConstraintArchive constraint;
    constraint.source_id = edge.source_id;
    constraint.target_id = edge.target_id;
    constraint.relative_pose = edge.relative_pose;
    constraint.information = edge.information;
    switch (edge.kind) {
      case GraphConstraintKind::Sequential:
        constraint.category = ConstraintCategory::Sequential; break;
      case GraphConstraintKind::NearChain:
        constraint.category = ConstraintCategory::NearChain; break;
      case GraphConstraintKind::LoopClosure:
        constraint.category = ConstraintCategory::LoopClosure; break;
    }
    constraint.loop_closure = constraint.category == ConstraintCategory::LoopClosure;
    archive.constraints.push_back(constraint);
  }
  archive.solver.node_count = archive.scans.size();
  archive.solver.constraint_count = archive.constraints.size();
  archive.solver.loop_closure_count = snapshot.loop_closures;
  archive.solver.iterations = snapshot.solver.iterations;
  archive.solver.residual_blocks = snapshot.solver.residual_blocks;
  archive.solver.initial_cost = snapshot.solver.initial_cost;
  archive.solver.final_cost = snapshot.solver.final_cost;
  archive.solver.converged = snapshot.solver.status == SolverStatus::Converged;
  switch (snapshot.solver.status) {
    case SolverStatus::NotRun: archive.solver.status = SolverStatusArchive::NotRun; break;
    case SolverStatus::Converged: archive.solver.status = SolverStatusArchive::Converged; break;
    case SolverStatus::Failed: archive.solver.status = SolverStatusArchive::Failed; break;
  }
  return archive;
}

}  // namespace

MappingRuntime::MappingRuntime(const SlamConfig &config, RuntimeOptions options)
  : config_(config), options_(std::move(options)), odometry_(config.odom_buffer_duration), mapper_(config)
{
  if (!options_.pose_log_path.empty()) {
    const auto parent = options_.pose_log_path.parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
    pose_log_.open(options_.pose_log_path, std::ios::out | std::ios::trunc);
    if (!pose_log_) throw std::runtime_error("cannot open pose quality log: " +
                                             options_.pose_log_path.string());
  }
  archive_.source_tag="slam_toolbox-2.6.9";
  archive_.config_hash="reference-v1";
  archive_.laser.range_min=config.min_laser_range; archive_.laser.range_max=config.max_laser_range;
  archive_.laser.extrinsic_x=config.lidar_extrinsic.x; archive_.laser.extrinsic_y=config.lidar_extrinsic.y;
  archive_.laser.extrinsic_yaw=config.lidar_extrinsic.yaw;
  localization_mode_=config.mode=="localization";
  if (config.mode=="continue_mapping" || localization_mode_) {
    archive_=LoadArchive(options_.map_prefix.string()+".posegraph.dora");
    mapper_.Restore(archive_);
  }
  EmitStatus("ready");
}

void MappingRuntime::HandleInput(std::string_view id, std::string_view payload, double arrival_time)
{
  if (stopped_) throw std::logic_error("runtime already stopped");
  try {
    if (id=="Odometry") {
      const auto odometry = ParseOdometry(payload);
      odometry_.Push(odometry);
      if (localization_mode_) EmitPredictedPose(odometry);
    }
    else if (id=="LaserScan") {
      auto scan=ParseLaserScan(payload);
      if (!archive_.laser.angle_increment) { archive_.laser.angle_min=scan.angle_min; archive_.laser.angle_max=scan.angle_max; archive_.laser.angle_increment=scan.angle_increment; }
      pending_.push_back({std::move(scan),arrival_time});
    } else if (id=="InitialPose") {
      if (!localization_mode_) throw std::logic_error("InitialPose requires localization mode");
      pose_extrapolator_.Reset();
      mapper_.SetInitialPose(ParseInitialPose(payload));
      localization_initialized_ = true;
      EmitStatus("relocalizing", "initial_pose_received");
    } else if (id=="SaveMap") { Save(); EmitStatus("saved"); }
    Drain(arrival_time);
  } catch (const std::exception &e) {
    EmitStatus("error",e.what());
    Emit("SafetyStop",nlohmann::json({{"stop",true},{"reason",e.what()}}).dump());
  }
}

void MappingRuntime::Tick(double now) { if (!stopped_) Drain(now); }

void MappingRuntime::Drain(double now)
{
  while (!pending_.empty()) {
    auto &p=pending_.front();
    const auto start=odometry_.Status(p.scan.start_stamp);
    if (start==BracketStatus::Ready) {
      LaserScan scan=std::move(p.scan); pending_.pop_front(); ProcessReady(scan); continue;
    }
    if (start==BracketStatus::TooOld || now-p.arrival_time>config_.max_sync_wait) {
      const std::string reason=start==BracketStatus::TooOld?"scan_older_than_odom_buffer":"odom_bracket_timeout";
      pending_.pop_front(); EmitStatus("scan_dropped",reason); continue;
    }
    break;
  }
}

void MappingRuntime::ProcessReady(const LaserScan &scan)
{
  const auto prepared=PrepareFixedBeamScan(scan,odometry_,config_.lidar_extrinsic,450);
  if (!prepared.ok()) { EmitStatus("scan_dropped",prepared.drop_reason); return; }
  if (localization_mode_ && !localization_initialized_) {
    EmitStatus("waiting_initial_pose", "initial_pose_required");
    return;
  }
  const auto result=localization_mode_?mapper_.Localize(prepared):mapper_.Process(prepared);
  if (localization_mode_ && result.skipped) return;
  if (localization_mode_ && !result.accepted) {
    localization_health_.ObserveRejected();
    EmitStatus("scan_rejected", "karto_not_processed");
    return;
  }
  if (!result.accepted) {
    EmitStatus("scan_rejected", "karto_not_processed");
    return;
  }
  if (localization_mode_) {
    localization_health_.ObserveMatched(result.match_response);
    if (result.match_response < 0.35) {
      EmitStatus("localization_lost","low_match_response");
      if (localization_health_.safety_stop())
      Emit("SafetyStop",nlohmann::json({{"stop",true},{"reason","sustained_localization_loss"}}).dump());
      return;
    }
    pose_extrapolator_.ObserveMatch(result.corrected_pose, prepared.base_pose);
    last_match_response_ = result.match_response;
  }
  if (!localization_mode_) {
  ScanArchive stored; stored.id=static_cast<std::int64_t>(archive_.scans.size()); stored.stamp=prepared.stamp;
  stored.odometric_pose=prepared.base_pose; stored.optimized_pose=result.corrected_pose; stored.points=prepared.base_points;
  archive_.scans.push_back(std::move(stored));
  archive_.solver.node_count=archive_.scans.size(); archive_.solver.constraint_count=result.constraint_count;
  }
  nlohmann::json pose={{"header",{{"stamp",prepared.stamp},{"frame_id","map"}}},
    {"pose",{{"x",result.corrected_pose.x},{"y",result.corrected_pose.y},{"yaw",result.corrected_pose.yaw}}},
    {"match_response",result.match_response},{"keyframe",result.keyframe},
    {"predicted",false}};
  if (pose_log_) {
    if (pose_log_samples_ == 0) first_logged_pose_ = result.corrected_pose;
    last_logged_pose_ = result.corrected_pose;
    ++pose_log_samples_;
    pose_log_ << nlohmann::json({{"type","pose"},{"stamp",prepared.stamp},
      {"pose",{{"x",result.corrected_pose.x},{"y",result.corrected_pose.y},
                {"yaw",result.corrected_pose.yaw}}},
      {"match_response",result.match_response},
      {"covariance",{{"xx",result.covariance_xx},{"yy",result.covariance_yy},
                      {"yaw_yaw",result.covariance_yaw_yaw}}},
      {"lost_count",localization_health_.consecutive_losses()}}).dump() << '\n';
    pose_log_.flush();
  }
  Emit("CorrectedPose",pose.dump());
  EmitStatus(localization_mode_?"localized":"mapping");
}

void MappingRuntime::EmitPredictedPose(const TimedPose2d &odometry)
{
  const auto predicted = pose_extrapolator_.Predict(odometry.pose);
  if (!predicted) return;
  nlohmann::json pose={{"header",{{"stamp",odometry.stamp},{"frame_id","map"}}},
    {"pose",{{"x",predicted->x},{"y",predicted->y},{"yaw",predicted->yaw}}},
    {"match_response",last_match_response_},{"keyframe",false},{"predicted",true}};
  Emit("CorrectedPose",pose.dump());
}

void MappingRuntime::Save()
{
  const GraphSnapshot snapshot = mapper_.Snapshot();
  if (snapshot.nodes.empty()) throw std::runtime_error("cannot save map without accepted scans");
  PoseGraphArchive saved_archive = ArchiveFromSnapshot(snapshot, archive_);
  if (saved_archive.solver.node_count != saved_archive.scans.size() ||
      saved_archive.solver.constraint_count != saved_archive.constraints.size())
    throw std::runtime_error("Karto snapshot/archive count mismatch");
  SaveArchiveAtomic(options_.map_prefix.string()+".posegraph.dora",saved_archive);
  GridConfig grid_config; grid_config.resolution=config_.map_resolution; grid_config.crop_margin=config_.map_crop_margin;
  SaveMapAtomic(options_.map_prefix,BuildOccupancyGrid(saved_archive,grid_config),saved_archive);
  archive_ = std::move(saved_archive);
}

void MappingRuntime::Stop()
{
  if (stopped_) return;
  if (pose_log_) {
    pose_log_ << nlohmann::json({{"type","summary"},
      {"samples",pose_log_samples_},{"loss_events",localization_health_.loss_events()},
      {"rejected_scans",localization_health_.rejected_scans()},
      {"start",{{"x",first_logged_pose_.x},{"y",first_logged_pose_.y},
                  {"yaw",first_logged_pose_.yaw}}},
      {"end",{{"x",last_logged_pose_.x},{"y",last_logged_pose_.y},
                {"yaw",last_logged_pose_.yaw}}}}).dump() << '\n';
    pose_log_.flush();
  }
  try { if (!localization_mode_ && !archive_.scans.empty()) Save(); EmitStatus("stopped"); }
  catch (const std::exception &e) { EmitStatus("error",e.what()); Emit("SafetyStop",nlohmann::json({{"stop",true},{"reason",e.what()}}).dump()); }
  stopped_=true;
}

void MappingRuntime::Emit(std::string id,std::string payload) { outputs_.push_back({std::move(id),std::move(payload)}); }
void MappingRuntime::EmitStatus(std::string state,std::string reason)
{
  Emit("SlamStatus",nlohmann::json({{"state",std::move(state)},{"reason",std::move(reason)},
    {"pending_scans",pending_.size()},{"processed_scans",archive_.scans.size()}}).dump());
}
std::vector<RuntimeOutput> MappingRuntime::TakeOutputs() { auto value=std::move(outputs_); outputs_.clear(); return value; }
std::size_t MappingRuntime::pending_scan_count() const noexcept { return pending_.size(); }
std::size_t MappingRuntime::processed_scan_count() const noexcept { return archive_.scans.size(); }

}  // namespace karto_dora
