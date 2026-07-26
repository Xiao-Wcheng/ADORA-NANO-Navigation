#include "nav2_rpp_port/path_handler.hpp"
#include <cassert>
#include <cmath>
#include <limits>

int main()
{
  using namespace nav2_rpp_port;
  Path2D path{{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}};
  PathHandler handler({3.0, 2.0, 0.01});
  auto result = handler.transformAndPrune(path, {1.1, 0, 0});
  assert(result.valid);
  assert(result.closest_index == 1);
  assert(result.pruned_count == 1);
  assert(result.local_path.front().x > -0.11 && result.local_path.front().x < -0.09);
  assert(result.local_path.back().x > result.local_path.front().x);

  result = handler.transformAndPrune({{0, 0, 0}, {0, 1, 0}}, {0, 0, 1.5707963267948966});
  assert(result.valid);
  assert(result.local_path.back().x > 0.99);
  assert(std::abs(result.local_path.back().y) < 1e-9);

  assert(!handler.transformAndPrune({}, {}).valid);
  path[1].x = std::numeric_limits<double>::quiet_NaN();
  assert(!handler.transformAndPrune(path, {}).valid);
}
