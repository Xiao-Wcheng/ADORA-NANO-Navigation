#pragma once

#include "karto_dora/archive.hpp"
#include "karto_dora/config.hpp"
#include "karto_dora/karto_mapper.hpp"
#include "karto_dora/localization_health.hpp"
#include "karto_dora/odometry_buffer.hpp"
#include "karto_dora/pose_extrapolator.hpp"

#include <deque>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <vector>

namespace karto_dora {

struct RuntimeOptions {
  std::filesystem::path map_prefix{"map"};
  std::filesystem::path pose_log_path;
};
struct RuntimeOutput { std::string id; std::string payload; };

class MappingRuntime {
 public:
  MappingRuntime(const SlamConfig &config, RuntimeOptions options);
  void HandleInput(std::string_view id, std::string_view payload, double arrival_time);
  void Tick(double now);
  void Stop();
  std::vector<RuntimeOutput> TakeOutputs();
  std::size_t pending_scan_count() const noexcept;
  std::size_t processed_scan_count() const noexcept;

 private:
  struct PendingScan { LaserScan scan; double arrival_time; };
  void Drain(double now);
  void ProcessReady(const LaserScan &scan);
  void EmitPredictedPose(const TimedPose2d &odometry);
  void Emit(std::string id, std::string payload);
  void EmitStatus(std::string state, std::string reason = {});
  void Save();

  SlamConfig config_;
  RuntimeOptions options_;
  OdometryBuffer odometry_;
  PoseExtrapolator pose_extrapolator_;
  KartoMapper mapper_;
  std::deque<PendingScan> pending_;
  std::vector<RuntimeOutput> outputs_;
  PoseGraphArchive archive_;
  bool stopped_{false};
  bool localization_mode_{false};
  bool localization_initialized_{false};
  LocalizationHealth localization_health_{0.35, 5};
  double last_match_response_{0.0};
  std::size_t pose_log_samples_{0};
  Pose2d first_logged_pose_;
  Pose2d last_logged_pose_;
  std::ofstream pose_log_;
};

}  // namespace karto_dora
