//
// Created by User on 2019/7/29.
//

#include "particle_filter_t.hh"

#include <vector>
#include <random>
#include <iostream>

#include <internal/control_model/pi.h>

autolabor::particle_filter_t::
particle_filter_t(size_t size)
    : max_size(size),
      measure_vote(std::max(static_cast<size_t>(1), size / 4)),
      engine(random()) {}

autolabor::odometry_t<>
autolabor::particle_filter_t::
operator()(const odometry_t<> &state) const {
    // 追踪完全丢失
    if (states.empty()) return {0, 0, NAN, NAN, NAN};
    
    // 移动粒子
    auto delta = state - e;
    
    std::vector<odometry_t<>> copy(states.size());
    std::transform(states.begin(), states.end(),
                   copy.begin(),
                   [&](const odometry_t<> &item) {
                       return item + delta;
                   });
    
    // 计算方差
    auto e_x      = .0,
         e_y      = .0,
         e_theta  = .0,
         e_theta2 = .0;
    
    for (const auto &item : copy) {
        e_x += item.x;
        e_y += item.y;
        e_theta += item.theta;
        e_theta2 += item.theta * item.theta;
    }
    
    const auto n = states.size();
    e_x /= n;
    e_y /= n;
    e_theta /= n;
    e_theta2 /= n;
    
    const auto d_theta = e_theta2 - e_theta * e_theta;
    return {0, 0, e_x, e_y, d_theta > 0.05 ? NAN : e_theta};
}

autolabor::odometry_t<>
autolabor::particle_filter_t::
update(const odometry_t<> &state,
       const Eigen::Vector2d &measure) {
    // 追踪丢失，重新初始化
    if (states.empty()) {
        save = state;
        auto        step = 2 * M_PI / max_size;
        for (size_t i    = 0; i < max_size; ++i)
            states.push_back({0, 0, measure[0], measure[1], i * step});
        return {0, 0, measure[0], measure[1], NAN};
    }
    
    // 计算控制量
    auto delta = state - save;
    
    // 过滤
    if (Eigen::Vector2d{delta.x, delta.y}.norm() < update_step)
        return operator()(state);
    save = state;
    
    // 按控制量更新粒子群
    for (auto &item : states) item += delta;
    
    // 排除所有异常粒子
    states.remove_if([measure](const odometry_t<> &item) {
        return (Eigen::Vector2d{item.x, item.y} - measure).norm() > accept_range;
    });
    
    // 剩余粒子数量
    const auto remain = states.size();
    
    // 找到最小和最大方向角
    auto e_theta2 = .0;
    e = {};
    for (const auto &item : states) {
        if (std::isnan(item.theta)) {
            std::cout << "look!" << std::endl;
            continue;
        }
        e.x += item.x;
        e.y += item.y;
        e.theta += item.theta;
        e_theta2 += item.theta * item.theta;
    }
    
    const auto n = static_cast<double>(remain + measure_vote);
    e.x = (e.x + measure_vote * measure[0]) / n;
    e.y = (e.y + measure_vote * measure[1]) / n;
    e.theta /= remain;
    e_theta2 /= remain;
    
    const auto d_theta = e_theta2 - e.theta * e.theta;
    std::cout << "remain = " << remain << ", D[θ] = " << d_theta << std::endl;
    
    // 重采样
    if (remain < max_size) {
        // 生成正态分布随机数
        std::normal_distribution<> spreader(e.theta, std::sqrt(d_theta));
        
        for (auto i = static_cast<long>(max_size - remain); i > 0; --i)
            states.push_back({0, 0, e.x, e.y, spreader(engine)});
    }
    
    return {0, 0, e.x, e.y, d_theta > 0.05 ? NAN : e.theta};
}
