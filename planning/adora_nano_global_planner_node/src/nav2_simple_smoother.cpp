// Copyright (c) 2022, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// Standalone adaptation of Nav2 SimpleSmoother:
// https://github.com/ros-navigation/navigation2/blob/main/nav2_smoother/src/simple_smoother.cpp

#include "nav2_simple_smoother.h"

#include <cmath>
#include <stdexcept>

Nav2SimpleSmoother::Nav2SimpleSmoother(
    const MapLoader &map, Nav2SimpleSmootherConfig config)
    : map_(map), config_(config) {
  if (config_.tolerance <= 0.0 || config_.max_iterations <= 0 ||
      config_.data_weight < 0.0 || config_.smooth_weight < 0.0 ||
      config_.refinement_count < 0) {
    throw std::invalid_argument("invalid Nav2 SimpleSmoother configuration");
  }
}

std::vector<GridPathPoint> Nav2SimpleSmoother::smooth(
    const std::vector<Point> &path) const {
  std::vector<GridPathPoint> result;
  result.reserve(path.size());
  for (const auto &point : path) {
    result.push_back(
        {static_cast<double>(point.x), static_cast<double>(point.y)});
  }
  if (result.size() <= 3) return result;

  result = smoothImpl(result);
  if (config_.do_refinement) {
    for (int i = 0; i < config_.refinement_count; ++i) {
      result = smoothImpl(result);
    }
  }
  return result;
}

std::vector<GridPathPoint> Nav2SimpleSmoother::smoothImpl(
    const std::vector<GridPathPoint> &path) const {
  std::vector<GridPathPoint> new_path = path;
  std::vector<GridPathPoint> last_path = path;
  double change = config_.tolerance;
  int iterations = 0;

  while (change >= config_.tolerance) {
    if (++iterations >= config_.max_iterations) return last_path;
    change = 0.0;

    for (std::size_t i = 1; i + 1 < path.size(); ++i) {
      const GridPathPoint before = new_path[i];
      new_path[i].x +=
          config_.data_weight * (path[i].x - new_path[i].x) +
          config_.smooth_weight *
              (new_path[i + 1].x + new_path[i - 1].x - 2.0 * new_path[i].x);
      new_path[i].y +=
          config_.data_weight * (path[i].y - new_path[i].y) +
          config_.smooth_weight *
              (new_path[i + 1].y + new_path[i - 1].y - 2.0 * new_path[i].y);
      change += std::abs(new_path[i].x - before.x) +
                std::abs(new_path[i].y - before.y);

      if (!collisionFree(new_path[i])) return last_path;
    }
    last_path = new_path;
  }
  return new_path;
}

bool Nav2SimpleSmoother::collisionFree(const GridPathPoint &point) const {
  const int x = static_cast<int>(std::lround(point.x));
  const int y = static_cast<int>(std::lround(point.y));
  return map_.isValid(x, y) && !map_.isOccupied(x, y);
}
