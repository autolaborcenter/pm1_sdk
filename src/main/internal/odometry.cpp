//
// Created by User on 2019/7/25.
//

#include <cmath>
#include "odometry.h"

autolabor::odometry_t<autolabor::odometry_type::delta>
autolabor::pm1::wheels_to_odometry(
    double left,
    double right,
    const chassis_config_t &config) {
    const auto l = config.r_left * left,
               r = config.r_right * right,
               s = (r + l) / 2,
               a = (r - l) / config.width;
    double     x, y;
    if (std::abs(a) < std::numeric_limits<double>::epsilon()) {
        x = s;
        y = 0;
    } else {
        auto _r = s / a;
        x = _r * std::sin(a);
        y = _r * (1 - std::cos(a));
    }
    return {std::abs(s), std::abs(a), x, y, a};
}
