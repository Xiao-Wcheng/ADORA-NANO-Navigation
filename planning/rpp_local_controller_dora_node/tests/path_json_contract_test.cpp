#include "nav2_rpp_port/path_json.hpp"

#include <cassert>
#include <cmath>
#include <nlohmann/json.hpp>

int main() {
  const nlohmann::json message = {
      {"path_found", true},
      {"waypoints",
       {{{"x", 17.5}, {"y", 23.25}}, {{"x", 18.75}, {"y", 24.5}}}}};
  const nav2_rpp_port::PathGridTransform transform{
      0.05, 85, -0.8, -3.1};
  const auto path = nav2_rpp_port::readPathJson(message, transform);

  assert(path.size() == 2);
  assert(std::abs(path[0].x - 0.1) < 1e-12);
  assert(std::abs(path[0].y - (-0.0375)) < 1e-12);
  assert(std::abs(path[1].x - 0.1625) < 1e-12);
  assert(std::abs(path[1].y - (-0.1)) < 1e-12);
  return 0;
}
