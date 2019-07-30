//
// Created by User on 2019/7/29.
//

#include "particle_filter_t.hh"

#include <internal/control_model/pi.h>
#include <vector>


autolabor::particle_filter_t::
particle_filter_t(size_t size) : max_size(size) {}

autolabor::odometry_t<>
autolabor::particle_filter_t::
operator()(const odometry_t<> &state) const {
    if (states.empty())
        // 追踪完全丢失
        return {0, 0, NAN, NAN, NAN};
    
    // 移动粒子
    auto                      delta = state - save;
    std::vector<odometry_t<>> copy(states.size());
    std::transform(states.begin(), states.end(),
                   copy.begin(),
                   [&](const odometry_t<> &item) {
                       return item + delta;
                   });
    
    // 找到最小和最大方向角
    auto min     = std::numeric_limits<double>::max(),
         max     = std::numeric_limits<double>::min(),
         e_x     = .0,
         e_y     = .0,
         e_theta = .0;
    
    for (const auto &item : copy) {
        e_x += item.x;
        e_y += item.y;
        e_theta += item.theta;
        if (item.theta < min) min = item.theta;
        if (item.theta > max) max = item.theta;
    }
    
    e_x /= states.size();
    e_y /= states.size();
    e_theta /= states.size();
    
    return {0, 0, e_x, e_y, max - min > M_PI / 3 ? NAN : e_theta};
}

autolabor::odometry_t<>
autolabor::particle_filter_t::
update(const odometry_t<> &state,
       const Eigen::Vector2d &measure) {
    // 计算控制量
    auto delta = state - save;
    save = state;
    
    // 按控制量更新粒子群
    for (auto &item : states) item += delta;
    
    // 排除所有异常粒子
    states.remove_if([&](const odometry_t<> &state) {
        return Eigen::Vector2d{state.x - measure[0], state.y - measure[1]}.norm() < 0.1;
    });
    
    // 计算
    const auto remain = states.size(),
               size   = max_size - remain;
    
    // 计算粒子共性
    double min, max, e_x, e_y, e_theta;
    if (states.empty()) {
        // 追踪丢失，重新初始化
        min     = 0;
        max     = 2 * M_PI;
        e_x     = measure[0];
        e_y     = measure[1];
        e_theta = (min + max) / 2;
    } else {
        // 找到最小和最大方向角
        min     = std::numeric_limits<double>::max();
        max     = std::numeric_limits<double>::min();
        e_x     = 0;
        e_y     = 0;
        e_theta = 0;
        for (const auto &item : states) {
            if (item.theta < min) min = item.theta;
            if (item.theta > max) max = item.theta;
            e_x += item.x;
            e_y += item.y;
            e_theta += item.theta;
        }
        e_x     = (e_x + 5 * measure[0]) / (remain + 5);
        e_y     = (e_y + 5 * measure[1]) / (remain + 5);
        e_theta /= states.size();
    }
    
    // 重采样
    if (size > 0)
        for (auto step  = (max - min) / size,
                  value = min;
             value < max;
             value += step) // NOLINT(cert-flp30-c)
            states.push_back({0, 0, e_x, e_y, value});
    
    return {0, 0, e_x, e_y, max - min > M_PI / 3 ? NAN : e_theta};
}

