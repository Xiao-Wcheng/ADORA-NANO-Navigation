#include "karto_dora/archive.hpp"

#include <nlohmann/json.hpp>
#include <openssl/sha.h>

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <unistd.h>

namespace karto_dora {
namespace {

using Json = nlohmann::json;

Json PoseJson(const Pose2d &p) { return Json::array({p.x, p.y, p.yaw}); }
Pose2d ReadPose(const Json &j)
{
  if (!j.is_array() || j.size() != 3) throw std::runtime_error("invalid pose");
  return {j.at(0).get<double>(), j.at(1).get<double>(), j.at(2).get<double>()};
}

const char *CategoryName(ConstraintCategory category)
{
  switch (category) {
    case ConstraintCategory::Sequential: return "sequential";
    case ConstraintCategory::NearChain: return "near_chain";
    case ConstraintCategory::LoopClosure: return "loop_closure";
  }
  throw std::runtime_error("invalid constraint category");
}

ConstraintCategory ReadCategory(const std::string &value)
{
  if (value == "sequential") return ConstraintCategory::Sequential;
  if (value == "near_chain") return ConstraintCategory::NearChain;
  if (value == "loop_closure") return ConstraintCategory::LoopClosure;
  throw std::runtime_error("invalid constraint category: " + value);
}

const char *StatusName(SolverStatusArchive status)
{
  switch (status) {
    case SolverStatusArchive::NotRun: return "not_run";
    case SolverStatusArchive::Converged: return "converged";
    case SolverStatusArchive::Failed: return "failed";
  }
  throw std::runtime_error("invalid solver status");
}

SolverStatusArchive ReadStatus(const std::string &value)
{
  if (value == "not_run") return SolverStatusArchive::NotRun;
  if (value == "converged") return SolverStatusArchive::Converged;
  if (value == "failed") return SolverStatusArchive::Failed;
  throw std::runtime_error("invalid solver status: " + value);
}

Json PayloadJson(const PoseGraphArchive &a)
{
  Json payload;
  payload["constraints"] = Json::array();
  for (const auto &c : a.constraints) {
    payload["constraints"].push_back({
      {"category", CategoryName(c.category)}, {"information", c.information},
      {"loop_closure", c.category == ConstraintCategory::LoopClosure},
      {"relative_pose", PoseJson(c.relative_pose)}, {"source_id", c.source_id},
      {"target_id", c.target_id}});
  }
  payload["laser"] = {
    {"angle_increment", a.laser.angle_increment}, {"angle_max", a.laser.angle_max},
    {"angle_min", a.laser.angle_min}, {"extrinsic_x", a.laser.extrinsic_x},
    {"extrinsic_y", a.laser.extrinsic_y}, {"extrinsic_yaw", a.laser.extrinsic_yaw},
    {"range_max", a.laser.range_max}, {"range_min", a.laser.range_min}};
  payload["scans"] = Json::array();
  for (const auto &s : a.scans) {
    Json points = Json::array();
    for (const auto &p : s.points) points.push_back(Json::array({p.x, p.y}));
    payload["scans"].push_back({
      {"id", s.id}, {"odometric_pose", PoseJson(s.odometric_pose)},
      {"optimized_pose", PoseJson(s.optimized_pose)}, {"points", std::move(points)},
      {"stamp", s.stamp}});
  }
  payload["solver"] = {
    {"constraint_count", a.solver.constraint_count}, {"converged", a.solver.converged},
    {"final_cost", a.solver.final_cost}, {"initial_cost", a.solver.initial_cost},
    {"iterations", a.solver.iterations}, {"residual_blocks", a.solver.residual_blocks},
    {"status", StatusName(a.solver.status)},
    {"loop_closure_count", a.solver.loop_closure_count},
    {"node_count", a.solver.node_count}};
  return payload;
}

std::string Sha256(const std::string &data)
{
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char *>(data.data()), data.size(), digest);
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (unsigned char byte : digest) out << std::setw(2) << static_cast<int>(byte);
  return out.str();
}

void WriteAll(int fd, const std::string &data)
{
  std::size_t offset = 0;
  while (offset < data.size()) {
    const ssize_t count = ::write(fd, data.data() + offset, data.size() - offset);
    if (count < 0) {
      if (errno == EINTR) continue;
      throw std::system_error(errno, std::generic_category(), "write archive");
    }
    offset += static_cast<std::size_t>(count);
  }
}

}  // namespace

bool operator==(const LaserModelArchive &a, const LaserModelArchive &b)
{
  return a.range_min == b.range_min && a.range_max == b.range_max &&
    a.angle_min == b.angle_min && a.angle_max == b.angle_max &&
    a.angle_increment == b.angle_increment && a.extrinsic_x == b.extrinsic_x &&
    a.extrinsic_y == b.extrinsic_y && a.extrinsic_yaw == b.extrinsic_yaw;
}
bool operator==(const ScanArchive &a, const ScanArchive &b)
{
  if (a.id != b.id || a.stamp != b.stamp || a.odometric_pose.x != b.odometric_pose.x ||
      a.odometric_pose.y != b.odometric_pose.y || a.odometric_pose.yaw != b.odometric_pose.yaw ||
      a.optimized_pose.x != b.optimized_pose.x || a.optimized_pose.y != b.optimized_pose.y ||
      a.optimized_pose.yaw != b.optimized_pose.yaw || a.points.size() != b.points.size()) return false;
  for (std::size_t i = 0; i < a.points.size(); ++i)
    if (a.points[i].x != b.points[i].x || a.points[i].y != b.points[i].y) return false;
  return true;
}
bool operator==(const ConstraintArchive &a, const ConstraintArchive &b)
{
  return a.source_id == b.source_id && a.target_id == b.target_id &&
    a.relative_pose.x == b.relative_pose.x && a.relative_pose.y == b.relative_pose.y &&
    a.relative_pose.yaw == b.relative_pose.yaw && a.information == b.information &&
    a.loop_closure == b.loop_closure && a.category == b.category;
}
bool operator==(const SolverMetadataArchive &a, const SolverMetadataArchive &b)
{
  return a.node_count == b.node_count && a.constraint_count == b.constraint_count &&
    a.loop_closure_count == b.loop_closure_count && a.initial_cost == b.initial_cost &&
    a.final_cost == b.final_cost && a.converged == b.converged &&
    a.iterations == b.iterations && a.residual_blocks == b.residual_blocks &&
    a.status == b.status;
}
bool operator==(const PoseGraphArchive &a, const PoseGraphArchive &b)
{
  return a.source_tag == b.source_tag && a.config_hash == b.config_hash &&
    a.laser == b.laser && a.scans == b.scans && a.constraints == b.constraints &&
    a.solver == b.solver;
}

void SaveArchiveAtomic(const std::filesystem::path &path, const PoseGraphArchive &archive)
{
  if (path.empty()) throw std::invalid_argument("archive path is empty");
  const Json payload = PayloadJson(archive);
  const std::string payload_text = payload.dump();
  Json root = {{"config_hash", archive.config_hash}, {"format", "dora-karto-posegraph"},
               {"payload", payload}, {"payload_checksum", Sha256(payload_text)},
               {"source_tag", archive.source_tag}, {"version", 2}};
  const std::string serialized = root.dump() + "\n";
  const auto temp = std::filesystem::path(path.string() + ".tmp");
  std::filesystem::create_directories(path.parent_path());
  int fd = ::open(temp.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd < 0) throw std::system_error(errno, std::generic_category(), "open archive temp");
  try {
    WriteAll(fd, serialized);
    if (::fsync(fd) != 0) throw std::system_error(errno, std::generic_category(), "fsync archive");
    if (::close(fd) != 0) throw std::system_error(errno, std::generic_category(), "close archive");
    fd = -1;
    std::filesystem::rename(temp, path);
    const int dir_fd = ::open(path.parent_path().c_str(), O_RDONLY | O_DIRECTORY);
    if (dir_fd >= 0) { ::fsync(dir_fd); ::close(dir_fd); }
  } catch (...) {
    if (fd >= 0) ::close(fd);
    std::error_code ignored;
    std::filesystem::remove(temp, ignored);
    throw;
  }
}

PoseGraphArchive LoadArchive(const std::filesystem::path &path)
{
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open pose graph archive: " + path.string());
  Json root;
  input >> root;
  if (root.value("format", "") != "dora-karto-posegraph")
    throw std::runtime_error("unsupported pose graph format");
  const int version = root.value("version", 0);
  if (version != 1 && version != 2) throw std::runtime_error("unsupported pose graph version");
  const Json &payload = root.at("payload");
  if (Sha256(payload.dump()) != root.at("payload_checksum").get<std::string>())
    throw std::runtime_error("pose graph checksum mismatch");

  PoseGraphArchive a;
  a.source_tag = root.at("source_tag").get<std::string>();
  a.config_hash = root.at("config_hash").get<std::string>();
  const auto &l = payload.at("laser");
  a.laser = {l.at("range_min"), l.at("range_max"), l.at("angle_min"), l.at("angle_max"),
             l.at("angle_increment"), l.at("extrinsic_x"), l.at("extrinsic_y"), l.at("extrinsic_yaw")};
  for (const auto &s : payload.at("scans")) {
    ScanArchive scan;
    scan.id = s.at("id"); scan.stamp = s.at("stamp");
    scan.odometric_pose = ReadPose(s.at("odometric_pose"));
    scan.optimized_pose = ReadPose(s.at("optimized_pose"));
    for (const auto &p : s.at("points")) scan.points.push_back({p.at(0), p.at(1)});
    a.scans.push_back(std::move(scan));
  }
  for (const auto &c : payload.at("constraints")) {
    ConstraintArchive constraint;
    constraint.source_id = c.at("source_id"); constraint.target_id = c.at("target_id");
    constraint.relative_pose = ReadPose(c.at("relative_pose"));
    constraint.information = c.at("information").get<std::array<double, 6>>();
    constraint.loop_closure = c.value("loop_closure", false);
    constraint.category = version == 1
      ? (constraint.loop_closure ? ConstraintCategory::LoopClosure : ConstraintCategory::Sequential)
      : ReadCategory(c.at("category").get<std::string>());
    constraint.loop_closure = constraint.category == ConstraintCategory::LoopClosure;
    a.constraints.push_back(constraint);
  }
  const auto &s = payload.at("solver");
  a.solver = {s.at("node_count"), s.at("constraint_count"), s.at("loop_closure_count"),
              s.at("initial_cost"), s.at("final_cost"), s.at("converged")};
  if (version == 2) {
    a.solver.iterations = s.at("iterations");
    a.solver.residual_blocks = s.at("residual_blocks");
    a.solver.status = ReadStatus(s.at("status").get<std::string>());
  } else {
    a.solver.status = SolverStatusArchive::NotRun;
  }
  return a;
}

}  // namespace karto_dora
