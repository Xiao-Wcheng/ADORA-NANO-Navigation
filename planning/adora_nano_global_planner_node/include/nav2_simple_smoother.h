// Copyright (c) 2022, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// Standalone adaptation of Nav2 SimpleSmoother:
// https://github.com/ros-navigation/navigation2/tree/main/nav2_smoother

#ifndef NAV2_SIMPLE_SMOOTHER_H
#define NAV2_SIMPLE_SMOOTHER_H

#include <vector>

#include "astar.h"
#include "map_loader.h"

struct GridPathPoint {
  double x{0.0};
  double y{0.0};

  bool operator==(const GridPathPoint &other) const {
    return x == other.x && y == other.y;
  }
};

struct Nav2SimpleSmootherConfig {
  double tolerance{1e-10};
  int max_iterations{1000};
  double data_weight{0.2};
  double smooth_weight{0.3};
  bool do_refinement{true};
  int refinement_count{2};
};

class Nav2SimpleSmoother {
 public:
  explicit Nav2SimpleSmoother(
      const MapLoader &map,
      Nav2SimpleSmootherConfig config = Nav2SimpleSmootherConfig{});

  std::vector<GridPathPoint> smooth(const std::vector<Point> &path) const;

 private:
  std::vector<GridPathPoint> smoothImpl(
      const std::vector<GridPathPoint> &path) const;
  bool collisionFree(const GridPathPoint &point) const;

  const MapLoader &map_;
  Nav2SimpleSmootherConfig config_;
};

#endif  // NAV2_SIMPLE_SMOOTHER_H
