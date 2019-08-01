//
// Created by User on 2019/7/29.
//

#include "particle_filter_t.hh"

#include <random>
#include <algorithm>
#include <iostream>
#include <filesystem>

#include <internal/control_model/pi.h>

autolabor::particle_filter_t::
particle_filter_t(size_t size)
    : max_size(size),
      measure_vote(std::max(static_cast<size_t>(1), size / 4)),
      engine(random()),
      match_save({}),
      e_save({0, 0, NAN, NAN, NAN}),
      e({}) {
    std::error_code _;
    std::filesystem::remove("particle_filter.txt", _);
    plot = std::ofstream("particle_filter.txt", std::ios::out);
}

autolabor::odometry_t<>
autolabor::particle_filter_t::
operator()(const odometry_t<> &state) const {
    if (!std::isnan(e_save.theta)) return e + (state - e_save);
    return {0, 0, NAN, NAN, NAN};
}

autolabor::odometry_t<>
autolabor::particle_filter_t::
update(const odometry_t<> &state,
       const Eigen::Vector2d &measure) {
    // 追踪丢失，重新初始化
    if (states.empty()) {
        initialize(state, measure);
        return {0, 0, NAN, NAN, NAN};
    }
    
    // 计算控制量
    auto delta = state - match_save;
    plot << state.x << ' '
         << state.y << ' '
         << measure[0] << ' '
         << measure[1] << ' '
         << Eigen::Vector2d{delta.x, delta.y}.norm() << ' '
         << (measure - measure_save).norm() << std::endl;
    
    // 过滤
    if (Eigen::Vector2d{delta.x, delta.y}.norm() < update_step)
        return operator()(state);
    match_save   = state;
    measure_save = measure;
    
    // 按控制量更新粒子群
    for (auto &item : states) item += delta;
    
    // 排除所有异常粒子
    states.remove_if([measure](const odometry_t<> &item) {
        return (Eigen::Vector2d{item.x, item.y} - measure).norm() > accept_range;
    });
    
    // 剩余粒子数量
    if (states.empty()) {
        initialize(state, measure);
        return {0, 0, NAN, NAN, NAN};
    }
    
    // 计算方差
    const auto remain   = states.size();
    auto       e_x      = .0,
               e_y      = .0,
               e_theta  = .0,
               e_theta2 = .0;
    
    for (const auto &item : states) {
        if (std::isnan(item.theta)) {
            std::cout << "look!" << std::endl;
            continue;
        }
        e_x += item.x;
        e_y += item.y;
        e_theta += item.theta;
        e_theta2 += item.theta * item.theta;
    }
    
    const auto n = static_cast<double>(remain + measure_vote);
    e_x = (e_x + measure_vote * measure[0]) / n;
    e_y = (e_y + measure_vote * measure[1]) / n;
    e_theta /= remain;
    e_theta2 /= remain;
    
    const auto d_theta = e_theta2 - e_theta * e_theta;
    
    // 重采样
    if (remain < max_size) {
        // 生成正态分布随机数
        std::normal_distribution<>
            spreader(e_theta, std::clamp(d_theta, d_range, 4.0));
        
        for (auto i = static_cast<long>(max_size - remain); i > 0; --i)
            states.push_back({0, 0, e_x, e_y, spreader(engine)});
    }
    
    odometry_t<> result{};
    
    if (d_theta < d_range) {
        e_save = state;
        result = e = {0, 0, e_x, e_y, e_theta};
    } else if (!std::isnan(e_save.theta))
        result = e + (state - e_save);
    else
        result = {0, 0, NAN, NAN, NAN};
    
    std::cout << "remain = " << remain
              << ", D[θ] = " << d_theta
              << std::endl;
    
    //    plot << state.x << ' '
    //         << state.y << ' '
    //         << measure[0] << ' '
    //         << measure[1] << ' '
    //         << remain << ' '
    //         << d_theta << ' '
    //         << result.x << ' '
    //         << result.y << ' '
    //         << result.theta << ' ';
    
    //    for (const auto &item : states) {
    //        plot << item.x << ' ' << item.y << ' ' << item.theta;
    //        std::cout << item.x << ' ' << item.y << ' ' << item.theta << std::endl;
    //    }
    plot << std::endl;
    std::cout << std::endl;
    
    plot.flush();
    return result;
}

void
autolabor::particle_filter_t::
initialize(const autolabor::odometry_t<> &state,
           const Eigen::Vector2d &measure) {
    measure_save = measure;
    match_save   = state;
    auto        step = 2 * M_PI / max_size;
    for (size_t i    = 0; i < max_size; ++i)
        states.push_back({0, 0, measure[0], measure[1], i * step - M_PI});
}
