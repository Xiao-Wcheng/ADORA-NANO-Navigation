#pragma once

#include <algorithm>
#include <cmath>

struct SelfFilterConfig {
    double half_length{0.17};
    double half_width{0.17};
    double padding{0.01};
};

inline bool inside_robot_footprint(
    double base_x, double base_y, const SelfFilterConfig& config)
{
    if (!std::isfinite(base_x) || !std::isfinite(base_y)) return false;
    const double half_length =
        std::max(0.0, config.half_length) + std::max(0.0, config.padding);
    const double half_width =
        std::max(0.0, config.half_width) + std::max(0.0, config.padding);
    return std::abs(base_x) <= half_length && std::abs(base_y) <= half_width;
}
