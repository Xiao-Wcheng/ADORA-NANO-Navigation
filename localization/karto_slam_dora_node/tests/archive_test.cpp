#include "karto_dora/archive.hpp"

#include <nlohmann/json.hpp>
#include <openssl/sha.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;

namespace {

std::string Sha256(const std::string &data)
{
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char *>(data.data()), data.size(), digest);
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (unsigned char byte : digest) out << std::setw(2) << static_cast<int>(byte);
  return out.str();
}

void Require(bool condition, const char *message)
{
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

template <typename Fn>
void RequireThrows(Fn fn, const char *message)
{
  try {
    fn();
  } catch (const std::exception &) {
    return;
  }
  Require(false, message);
}

karto_dora::PoseGraphArchive Fixture()
{
  karto_dora::PoseGraphArchive archive;
  archive.source_tag = "slam_toolbox-2.6.9";
  archive.config_hash = "cfg-123";
  archive.laser = {0.05, 8.0, -3.14, 3.14, 0.01, 0.09, 0.06, 0.0};
  archive.scans.push_back({1, 100.0, {0.1, 0.2, 0.01}, {0.11, 0.19, 0.0},
                           {{1.0, 0.0}, {0.0, 1.0}}});
  archive.scans.push_back({2, 100.1, {0.3, 0.2, 0.01}, {0.30, 0.20, 0.0},
                           {{0.8, 0.1}, {-0.1, 0.9}}});
  archive.constraints.push_back({1, 2, {0.19, 0.01, 0.0},
                                 {100.0, 0.0, 0.0, 100.0, 0.0, 200.0}, false});
  archive.constraints.back().category = karto_dora::ConstraintCategory::Sequential;
  archive.solver.node_count = 2;
  archive.solver.constraint_count = 1;
  archive.solver.loop_closure_count = 0;
  archive.solver.iterations = 4;
  archive.solver.residual_blocks = 1;
  archive.solver.initial_cost = 0.01;
  archive.solver.final_cost = 0.0001;
  archive.solver.status = karto_dora::SolverStatusArchive::Converged;
  return archive;
}

}  // namespace

int main()
{
  const fs::path root = fs::temp_directory_path() / "karto_dora_archive_test";
  fs::remove_all(root);
  fs::create_directories(root);
  const fs::path path = root / "map.posegraph.dora";

  const auto expected = Fixture();
  karto_dora::SaveArchiveAtomic(path, expected);
  {
    std::ifstream version_input(path);
    const std::string version_text((std::istreambuf_iterator<char>(version_input)), {});
    Require(version_text.find("\"version\":2") != std::string::npos,
            "new archives must use version 2");
    Require(version_text.find("\"category\":\"sequential\"") != std::string::npos,
            "constraint category was not serialized");
    Require(version_text.find("\"iterations\":4") != std::string::npos,
            "solver report was not serialized");
  }
  const auto actual = karto_dora::LoadArchive(path);
  Require(actual == expected, "archive round trip changed data");
  Require(!fs::exists(path.string() + ".tmp"), "temporary file was left behind");

  std::ifstream input(path);
  std::string valid((std::istreambuf_iterator<char>(input)), {});

  {
    auto legacy = nlohmann::json::parse(valid);
    legacy["version"] = 1;
    for (auto &constraint : legacy["payload"]["constraints"])
      constraint.erase("category");
    auto &solver = legacy["payload"]["solver"];
    solver.erase("iterations");
    solver.erase("residual_blocks");
    solver.erase("status");
    legacy["payload_checksum"] = Sha256(legacy["payload"].dump());
    const fs::path legacy_path = root / "legacy.posegraph.dora";
    std::ofstream legacy_output(legacy_path);
    legacy_output << legacy.dump() << '\n';
    legacy_output.close();
    const auto migrated = karto_dora::LoadArchive(legacy_path);
    Require(migrated.constraints.at(0).category ==
              karto_dora::ConstraintCategory::Sequential,
            "v1 non-loop constraint did not migrate to sequential");
    Require(migrated.solver.status == karto_dora::SolverStatusArchive::NotRun,
            "v1 missing solver report must migrate to not_run");
  }

  std::string corrupt = valid;
  const auto point = corrupt.find("0.19");
  Require(point != std::string::npos, "fixture value not found");
  corrupt.replace(point, 4, "9.99");
  {
    std::ofstream output(path, std::ios::trunc);
    output << corrupt;
  }
  RequireThrows([&] { karto_dora::LoadArchive(path); },
                "checksum corruption was accepted");

  {
    std::ofstream output(path, std::ios::trunc);
    output << valid;
  }
  std::string wrong_version = valid;
  const auto version = wrong_version.find("\"version\":2");
  Require(version != std::string::npos, "version field not deterministic");
  wrong_version.replace(version, 11, "\"version\":9");
  {
    std::ofstream output(path, std::ios::trunc);
    output << wrong_version;
  }
  RequireThrows([&] { karto_dora::LoadArchive(path); },
                "unknown archive version was accepted");

  // A stale/interrupted sibling temp must neither hide nor alter a valid archive.
  {
    std::ofstream output(path, std::ios::trunc);
    output << valid;
    std::ofstream temp(path.string() + ".tmp", std::ios::trunc);
    temp << "partial";
  }
  Require(karto_dora::LoadArchive(path) == expected,
          "stale temporary file affected valid archive");
  karto_dora::SaveArchiveAtomic(path, expected);
  Require(karto_dora::LoadArchive(path) == expected,
          "atomic replacement did not preserve archive");

  fs::remove_all(root);
  std::cout << "archive_test PASS\n";
}
