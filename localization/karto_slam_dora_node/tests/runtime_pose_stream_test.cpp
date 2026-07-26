#include "karto_dora/archive.hpp"
#include "karto_dora/runtime.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <sstream>

namespace fs = std::filesystem;

namespace {

void Require(bool ok, const char *message) {
  if (!ok) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

std::string Odom(double stamp, double x) {
  const auto sec = static_cast<long>(stamp);
  const auto ns = static_cast<long>((stamp - sec) * 1e9);
  std::ostringstream out;
  out << "{\"header\":{\"stamp\":{\"sec\":" << sec << ",\"nanosec\":" << ns
      << "}},\"pose\":{\"pose\":{\"position\":{\"x\":" << x
      << ",\"y\":0},\"orientation\":{\"z\":0,\"w\":1}}},"
      << "\"twist\":{\"twist\":{\"linear\":{\"x\":0.04,\"y\":0},"
      << "\"angular\":{\"z\":0}}}}";
  return out.str();
}

std::string Scan(double stamp) {
  const auto sec = static_cast<long>(stamp);
  const auto ns = static_cast<long>((stamp - sec) * 1e9);
  std::ostringstream ranges;
  for (int i = 0; i < 450; ++i) {
    if (i) ranges << ',';
    ranges << (2.0 + 0.2 * std::sin(i * 0.04));
  }
  std::ostringstream out;
  out << "{\"header\":{\"stamp\":{\"sec\":" << sec << ",\"nanosec\":" << ns
      << "},\"frame_id\":\"laser\"},\"angle_min\":0.0,\"angle_max\":6.269222673,"
      << "\"angle_increment\":0.013962634016,\"scan_time\":0.06,"
      << "\"time_increment\":0.000133333333,\"range_min\":0.05,\"range_max\":8.0,"
      << "\"ranges\":[" << ranges.str() << "]}";
  return out.str();
}

karto_dora::PoseGraphArchive Map() {
  karto_dora::PoseGraphArchive archive;
  archive.source_tag = "slam_toolbox-2.6.9";
  archive.config_hash = "reference-v1";
  archive.laser.range_min = 0.05;
  archive.laser.range_max = 8.0;
  archive.laser.angle_min = 0.0;
  archive.laser.angle_increment = 2.0 * M_PI / 450.0;
  archive.laser.angle_max = 2.0 * M_PI - archive.laser.angle_increment;
  archive.laser.extrinsic_x = 0.09;
  archive.laser.extrinsic_y = 0.06;
  karto_dora::ScanArchive scan;
  scan.id = 0;
  scan.stamp = 1.0;
  for (int i = 0; i < 450; ++i) {
    const double angle = i * archive.laser.angle_increment;
    const double range = 2.0 + 0.2 * std::sin(i * 0.04);
    scan.points.push_back({0.09 + range * std::cos(angle), 0.06 + range * std::sin(angle)});
  }
  archive.scans.push_back(std::move(scan));
  archive.solver.node_count = 1;
  return archive;
}

bool HasPredictedPose(const std::vector<karto_dora::RuntimeOutput> &outputs) {
  for (const auto &output : outputs) {
    if (output.id != "CorrectedPose") continue;
    const auto value = nlohmann::json::parse(output.payload);
    if (value.value("predicted", false)) return true;
  }
  return false;
}

bool HasAnyPose(const std::vector<karto_dora::RuntimeOutput> &outputs) {
  for (const auto &output : outputs) if (output.id == "CorrectedPose") return true;
  return false;
}

bool HasStatusReason(const std::vector<karto_dora::RuntimeOutput> &outputs,
                     const std::string &reason) {
  for (const auto &output : outputs) {
    if (output.id != "SlamStatus") continue;
    const auto value = nlohmann::json::parse(output.payload);
    if (value.value("reason", std::string{}) == reason) return true;
  }
  return false;
}

}  // namespace

int main() {
  const fs::path root = fs::temp_directory_path() / "karto_runtime_pose_stream_test";
  fs::remove_all(root);
  fs::create_directories(root);
  const fs::path prefix = root / "map";
  karto_dora::SaveArchiveAtomic(prefix.string() + ".posegraph.dora", Map());

  auto config = karto_dora::SlamConfig::ReferenceDefaults();
  config.mode = "localization";
  karto_dora::RuntimeOptions options;
  options.map_prefix = prefix;
  karto_dora::MappingRuntime runtime(config, options);
  runtime.TakeOutputs();

  runtime.HandleInput("Odometry", Odom(10.0, 0.0), 1.0);
  runtime.HandleInput("LaserScan", Scan(10.02), 1.01);
  runtime.HandleInput("Odometry", Odom(10.1, 0.01), 1.02);
  const auto before_initial_pose = runtime.TakeOutputs();
  Require(!HasAnyPose(before_initial_pose),
          "localization published a pose before explicit initialization");
  Require(HasStatusReason(before_initial_pose, "initial_pose_required"),
          "localization did not report that an initial pose is required");

  runtime.HandleInput("InitialPose", R"({"x":0.0,"y":0.0,"yaw":0.0})", 1.03);
  runtime.TakeOutputs();
  runtime.HandleInput("LaserScan", Scan(10.12), 1.04);
  runtime.HandleInput("Odometry", Odom(10.2, 0.02), 1.05);
  Require(HasAnyPose(runtime.TakeOutputs()),
          "explicitly initialized scan did not produce a matched pose");

  const auto scans_before_prediction = runtime.processed_scan_count();
  runtime.HandleInput("Odometry", Odom(10.25, 0.022), 1.06);
  Require(HasPredictedPose(runtime.TakeOutputs()),
          "odometry update did not produce a continuous predicted pose");
  Require(runtime.processed_scan_count() == scans_before_prediction,
          "pose prediction modified the persistent Karto graph");

  runtime.HandleInput("InitialPose", R"({"x":0.0,"y":0.0,"yaw":0.0})", 1.07);
  runtime.TakeOutputs();
  runtime.HandleInput("Odometry", Odom(10.3, 0.03), 1.08);
  Require(!HasAnyPose(runtime.TakeOutputs()),
          "prediction was not gated after an InitialPose request");

  runtime.Stop();
  fs::remove_all(root);
  std::cout << "runtime_pose_stream_test PASS\n";
}
