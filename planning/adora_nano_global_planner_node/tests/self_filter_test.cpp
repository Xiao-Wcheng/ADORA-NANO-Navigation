#include "self_filter.h"

#include <cassert>

int main()
{
    SelfFilterConfig config;
    config.half_length = 0.17;
    config.half_width = 0.17;
    config.padding = 0.01;
    assert(inside_robot_footprint(0.0, 0.0, config));
    assert(inside_robot_footprint(0.18, -0.18, config));
    assert(!inside_robot_footprint(0.181, 0.0, config));
    assert(!inside_robot_footprint(0.0, -0.181, config));
    return 0;
}
