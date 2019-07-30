//
// Created by User on 2019/7/29.
//

#include "particle_filter_t.hh"

#include <internal/control_model/pi.h>
#include <vector>
#include <iostream>


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
        if (item.theta < min) min = item.theta;
        if (item.theta > max) max = item.theta;
        e_x += item.x;
        e_y += item.y;
        e_theta += item.theta;
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
    
    // 过滤
    if (Eigen::Vector2d{delta.x, delta.y}.norm() < update_step)
        return operator()(state);
    save = state;
    
    // 按控制量更新粒子群
    for (auto &item : states) item += delta;
    
    // 排除所有异常粒子
    states.remove_if([&](const odometry_t<> &state) {
        std::cout << (Eigen::Vector2d{state.x, state.y} - measure).transpose() << std::endl;
        return (Eigen::Vector2d{state.x, state.y} - measure).norm() > accept_range;
    });
    
    // 剩余粒子数量
    const auto remain = states.size();
    
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
            if (std::isnan(item.theta)) {
                std::cout << "look!" << std::endl;
                continue;
            }
            if (item.theta < min) min = item.theta;
            if (item.theta > max) max = item.theta;
            e_x += item.x;
            e_y += item.y;
            e_theta += item.theta;
        }
    
        auto n = static_cast<double>(remain + measure_vote);
        e_x = (e_x + measure_vote * measure[0]) / n;
        e_y = (e_y + measure_vote * measure[1]) / n;
        e_theta /= remain;
    }
    
    std::cout << "max - min = " << max - min << std::endl;
    
    // 重采样
    if (max_size > remain) {
        auto step  = (max - min) / (max_size - remain),
             value = min - step;
        while (value < max)
            states.push_back({0, 0, e_x, e_y, value += step});
    }
    
    return {0, 0, e_x, e_y, max - min > M_PI / 3 ? NAN : e_theta};
}
