#include "karto_dora/occupancy_grid.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <system_error>

namespace karto_dora {
namespace {

Point2d Transform(const Pose2d &pose, const Point2d &point)
{
  const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  return {pose.x + c * point.x - s * point.y,
          pose.y + s * point.x + c * point.y};
}

void AtomicWrite(const std::filesystem::path &path, const std::string &data)
{
  const auto temp = std::filesystem::path(path.string() + ".tmp");
  std::ofstream output(temp, std::ios::binary | std::ios::trunc);
  if (!output) throw std::runtime_error("cannot create " + temp.string());
  output.write(data.data(), static_cast<std::streamsize>(data.size()));
  output.flush();
  if (!output) { output.close(); std::filesystem::remove(temp); throw std::runtime_error("map write failed"); }
  output.close();
  std::filesystem::rename(temp, path);
}

}  // namespace

std::int8_t OccupancyGrid::At(std::size_t x, std::size_t y) const
{
  if (x >= width || y >= height) throw std::out_of_range("grid cell");
  return cells[y * width + x];
}

std::int8_t OccupancyGrid::AtMetric(double x, double y) const
{
  const auto gx = static_cast<long>(std::floor((x - origin_x) / resolution));
  const auto gy = static_cast<long>(std::floor((y - origin_y) / resolution));
  if (gx < 0 || gy < 0) throw std::out_of_range("metric grid cell");
  return At(static_cast<std::size_t>(gx), static_cast<std::size_t>(gy));
}

OccupancyGrid BuildOccupancyGrid(const PoseGraphArchive &graph, const GridConfig &config)
{
  if (graph.scans.empty()) throw std::invalid_argument("cannot build map without scans");
  if (!(config.resolution > 0.0) || config.crop_margin < 0.0 ||
      !(config.free_threshold < config.occupied_threshold))
    throw std::invalid_argument("invalid grid configuration");

  double min_x = std::numeric_limits<double>::infinity();
  double min_y = min_x, max_x = -min_x, max_y = -min_x;
  for (const auto &scan : graph.scans) {
    min_x = std::min(min_x, scan.optimized_pose.x); max_x = std::max(max_x, scan.optimized_pose.x);
    min_y = std::min(min_y, scan.optimized_pose.y); max_y = std::max(max_y, scan.optimized_pose.y);
    for (const auto &local : scan.points) {
      const auto p = Transform(scan.optimized_pose, local);
      min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
      min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
    }
  }
  OccupancyGrid grid;
  grid.resolution = config.resolution;
  grid.origin_x = std::floor((min_x - config.crop_margin) / config.resolution) * config.resolution;
  grid.origin_y = std::floor((min_y - config.crop_margin) / config.resolution) * config.resolution;
  const double upper_x = std::ceil((max_x + config.crop_margin) / config.resolution) * config.resolution;
  const double upper_y = std::ceil((max_y + config.crop_margin) / config.resolution) * config.resolution;
  grid.width = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil((upper_x-grid.origin_x)/grid.resolution)));
  grid.height = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil((upper_y-grid.origin_y)/grid.resolution)));
  std::vector<unsigned> hits(grid.width * grid.height), misses(grid.width * grid.height);
  auto cell = [&](double x, double y) {
    return std::pair<int,int>{static_cast<int>(std::floor((x-grid.origin_x)/grid.resolution)),
                              static_cast<int>(std::floor((y-grid.origin_y)/grid.resolution))};
  };
  for (const auto &scan : graph.scans) {
    const auto start = cell(scan.optimized_pose.x, scan.optimized_pose.y);
    for (const auto &local : scan.points) {
      const auto world = Transform(scan.optimized_pose, local); const auto end = cell(world.x, world.y);
      int x=start.first, y=start.second, dx=std::abs(end.first-x), sx=x<end.first?1:-1;
      int dy=-std::abs(end.second-y), sy=y<end.second?1:-1, error=dx+dy;
      while (x != end.first || y != end.second) {
        if (x>=0 && y>=0 && x<static_cast<int>(grid.width) && y<static_cast<int>(grid.height))
          ++misses[static_cast<std::size_t>(y)*grid.width+static_cast<std::size_t>(x)];
        const int twice=2*error; if (twice>=dy) { error+=dy; x+=sx; } if (twice<=dx) { error+=dx; y+=sy; }
      }
      if (x>=0 && y>=0 && x<static_cast<int>(grid.width) && y<static_cast<int>(grid.height))
        ++hits[static_cast<std::size_t>(y)*grid.width+static_cast<std::size_t>(x)];
    }
  }
  grid.cells.assign(grid.width*grid.height, -1);
  for (std::size_t i=0;i<grid.cells.size();++i) {
    const unsigned total=hits[i]+misses[i]; if (!total) continue;
    const double probability=static_cast<double>(hits[i])/total;
    if (probability >= config.occupied_threshold) grid.cells[i]=100;
    else if (probability <= config.free_threshold) grid.cells[i]=0;
  }
  return grid;
}

SavedMap SaveMapAtomic(const std::filesystem::path &prefix, const OccupancyGrid &grid,
                       const PoseGraphArchive &graph)
{
  if (grid.cells.size()!=grid.width*grid.height) throw std::invalid_argument("invalid grid size");
  std::filesystem::create_directories(prefix.parent_path());
  SavedMap saved{prefix.string()+".pgm", prefix.string()+".yaml", prefix.string()+".metadata.json"};
  std::ostringstream pgm; pgm << "P5\n# dora karto map\n" << grid.width << ' ' << grid.height << "\n255\n";
  for (std::size_t row=grid.height; row-- > 0;) for (std::size_t x=0;x<grid.width;++x) {
    const auto value=grid.At(x,row); const unsigned char pixel=value<0?205:(value>=65?0:254);
    pgm.write(reinterpret_cast<const char*>(&pixel),1);
  }
  AtomicWrite(saved.pgm, pgm.str());
  std::ostringstream yaml; yaml << "image: " << saved.pgm.filename().string() << "\nresolution: " << grid.resolution
    << "\norigin: [" << grid.origin_x << ", " << grid.origin_y << ", 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\nmode: trinary\n";
  AtomicWrite(saved.yaml, yaml.str());
  std::size_t sequential=0, near_chain=0, loops=0;
  for (const auto &constraint:graph.constraints) {
    if (constraint.category==ConstraintCategory::LoopClosure) ++loops;
    else if (constraint.category==ConstraintCategory::NearChain) ++near_chain;
    else ++sequential;
  }
  const char *solver_status=graph.solver.status==SolverStatusArchive::Converged?"converged":
    (graph.solver.status==SolverStatusArchive::Failed?"failed":"not_run");
  double closure_distance=0.0, closure_heading=0.0;
  if (graph.scans.size()>1) {
    const auto &first=graph.scans.front().optimized_pose;
    const auto &last=graph.scans.back().optimized_pose;
    closure_distance=std::hypot(last.x-first.x,last.y-first.y);
    closure_heading=std::remainder(last.yaw-first.yaw,2.0*M_PI)*180.0/M_PI;
  }
  nlohmann::json meta={{"config_hash",graph.config_hash},{"constraints",graph.constraints.size()},
    {"height",grid.height},{"loop_closures",loops},{"near_chain_constraints",near_chain},
    {"sequential_constraints",sequential},{"resolution",grid.resolution},
    {"scans",graph.scans.size()},{"solver_status",solver_status},
    {"solver_iterations",graph.solver.iterations},{"initial_cost",graph.solver.initial_cost},
    {"final_cost",graph.solver.final_cost},{"closure_distance_m",closure_distance},
    {"closure_heading_deg",closure_heading},{"source_tag",graph.source_tag},{"width",grid.width}};
  AtomicWrite(saved.metadata, meta.dump(2)+"\n");
  return saved;
}

}  // namespace karto_dora
