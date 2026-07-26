#include "karto_dora/runtime.hpp"
#include "karto_dora/archive.hpp"

#include <nlohmann/json.hpp>

#include <cstdlib>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

namespace fs = std::filesystem;
namespace {
void Require(bool ok, const char *msg) { if (!ok) { std::cerr << "FAIL: " << msg << '\n'; std::exit(1); } }
std::string Odom(double stamp, double x)
{
  const auto sec=static_cast<long>(stamp); const auto ns=static_cast<long>((stamp-sec)*1e9);
  std::ostringstream s; s << "{\"header\":{\"stamp\":{\"sec\":"<<sec<<",\"nanosec\":"<<ns
    <<"}},\"pose\":{\"pose\":{\"position\":{\"x\":"<<x<<",\"y\":0},\"orientation\":{\"z\":0,\"w\":1}}},"
    <<"\"twist\":{\"twist\":{\"linear\":{\"x\":0,\"y\":0},\"angular\":{\"z\":0}}}}"; return s.str();
}
std::string Scan(long nanosec)
{
  std::ostringstream r; for(int i=0;i<450;++i) { if(i) r<<','; r << (2.0+0.2*std::sin(i*0.04)); }
  return "{\"header\":{\"stamp\":{\"sec\":10,\"nanosec\":"+std::to_string(nanosec)+"},\"frame_id\":\"laser\"},"
    "\"angle_min\":0.0,\"angle_max\":6.269222673,\"angle_increment\":0.013962634016,"
    "\"scan_time\":0.06,\"time_increment\":0.000133333333,\"range_min\":0.05,\"range_max\":8.0,\"ranges\":["+r.str()+"]}";
}
}  // namespace

int main()
{
  const fs::path root=fs::temp_directory_path()/"karto_runtime_test"; fs::remove_all(root); fs::create_directories(root);
  auto config=karto_dora::SlamConfig::ReferenceDefaults();
  config.minimum_travel_distance=0.01;
  karto_dora::RuntimeOptions options; options.map_prefix=root/"map";
  options.pose_log_path=root/"localization.jsonl";
  karto_dora::MappingRuntime runtime(config, options);
  runtime.HandleInput("Odometry", Odom(10.0,0.0), 1.0);
  runtime.HandleInput("LaserScan", Scan(20000000), 1.01);
  Require(runtime.pending_scan_count()==1, "scan should wait for future odometry");
  runtime.HandleInput("Odometry", Odom(10.1,0.02), 1.02);
  Require(runtime.pending_scan_count()==0 && runtime.processed_scan_count()==1, "bracket-ready scan was not processed");
  runtime.HandleInput("LaserScan", Scan(120000000), 1.03);
  runtime.HandleInput("Odometry", Odom(10.2,0.20), 1.04);
  Require(runtime.processed_scan_count()==2, "second scan was not processed");
  bool pose=false,status=false; for(const auto &o:runtime.TakeOutputs()){ pose|=o.id=="CorrectedPose"; status|=o.id=="SlamStatus"; }
  Require(pose && status, "runtime outputs missing");
  runtime.Stop();
  Require(fs::exists(options.pose_log_path), "pose quality log was not created");
  std::ifstream pose_log(options.pose_log_path);
  std::string line;
  bool saw_pose=false, saw_summary=false;
  while(std::getline(pose_log,line)) {
    const auto record=nlohmann::json::parse(line);
    if(record.value("type","")=="pose") {
      saw_pose=true;
      Require(record.contains("stamp") && record.contains("pose") &&
              record.contains("match_response") && record.contains("covariance"),
              "pose quality record is incomplete");
    }
    saw_summary|=record.value("type","")=="summary";
  }
  Require(saw_pose && saw_summary,"pose quality log lacks pose or summary records");
  Require(fs::exists(root/"map.posegraph.dora") && fs::exists(root/"map.pgm") && fs::exists(root/"map.yaml"), "stop did not save final map");
  const auto archive=karto_dora::LoadArchive(root/"map.posegraph.dora");
  Require(std::abs(archive.scans.at(0).odometric_pose.x-0.004)<1e-6,
          "saved graph node is not the scan-start base pose");
  Require(std::abs(archive.scans.at(0).points.at(0).x-2.09)<1e-6 &&
          std::abs(archive.scans.at(0).points.at(0).y-0.06)<1e-6,
          "saved local point does not contain the lidar extrinsic");
  Require(archive.constraints.size()>0, "runtime saved no real Karto constraints");
  std::ifstream meta_input(root/"map.metadata.json");
  nlohmann::json meta; meta_input >> meta;
  Require(meta.at("scans")==archive.scans.size(), "metadata scan count mismatch");
  Require(meta.at("constraints")==archive.constraints.size(), "metadata constraint count mismatch");
  Require(meta.at("loop_closures")==archive.solver.loop_closure_count, "metadata loop count mismatch");
  fs::remove_all(root); std::cout << "runtime_mapping_test PASS\n";
}
