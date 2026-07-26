#include "karto_dora/config.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
  const auto config = karto_dora::SlamConfig::ReferenceDefaults();
  assert(std::abs(config.minimum_travel_distance - 0.05) < 1e-12);
  assert(std::abs(config.minimum_travel_heading - 0.05) < 1e-12);
  assert(std::abs(config.correlation_search_space_dimension - 0.5) < 1e-12);
  assert(std::abs(config.correlation_search_space_resolution - 0.01) < 1e-12);
  assert(std::abs(config.loop_search_maximum_distance - 3.0) < 1e-12);
  assert(config.scan_buffer_size == 10);
  assert(std::abs(config.scan_buffer_maximum_scan_distance - 10.0) < 1e-12);
  assert(std::abs(config.max_laser_range - 8.0) < 1e-12);
  assert(std::abs(config.lidar_extrinsic.x - 0.09) < 1e-12);
  assert(std::abs(config.lidar_extrinsic.y - 0.06) < 1e-12);
  config.Validate();
  std::cout << "config_test PASS\n";
}
