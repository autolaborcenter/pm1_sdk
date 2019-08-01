//
// Created by User on 2019/7/29.
//

#include "particle_filter_t.hh"

#include <random>
#include <algorithm>
#include <filesystem>

#include <internal/control_model/pi.h>
#include <numeric>

autolabor::particle_filter_t::
particle_filter_t(size_t size)
    : states(size),
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

constexpr auto epsilon = std::numeric_limits<float>::epsilon();

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
    
    // 计算权重
    std::vector<double> weights(states.size());
    std::transform(states.begin(), states.end(),
                   weights.begin(),
                   [measure](const odometry_t<> &item) {
                       const auto distance = (Eigen::Vector2d{item.x, item.y} - measure).norm();
                       return std::clamp(1 - distance / accept_range, 0.0, 1.0);
                   });
    
    auto sum = std::accumulate(weights.begin(), weights.end(), .0);
    
    // 剩余粒子数量
    if (sum < epsilon) {
        initialize(state, measure);
        return {0, 0, NAN, NAN, NAN};
    }
    
    // 计算方差
    auto e_x      = .0,
         e_y      = .0,
         e_theta  = .0,
         e_theta2 = .0;
    
    auto            weight = weights.cbegin();
    for (const auto &item : states) {
        auto k = *weight++;
        e_x += k * item.x;
        e_y += k * item.y;
        e_theta += k * item.theta;
        e_theta2 += k * item.theta * item.theta;
    }
    
    e_x = (e_x + measure_weight * measure[0]) / (sum + measure_weight);
    e_y = (e_y + measure_weight * measure[1]) / (sum + measure_weight);
    e_theta /= sum;
    e_theta2 /= sum;
    
    const auto d_theta = e_theta2 - e_theta * e_theta;
    
    // 生成正态分布随机数
    std::normal_distribution<>
        spreader(e_theta, std::sqrt(std::clamp(d_theta, d_range, 4.0)));
    
    // 重采样
    weight = weights.cbegin();
    for (auto &item : states) if (*weight++ < 0.2) item = {0, 0, e_x, e_y, spreader(engine)};
    
    // 计算位姿
    odometry_t<> result{};
    if (d_theta < d_range) {
        e_save = state;
        result = e = {0, 0, e_x, e_y, e_theta};
    } else if (!std::isnan(e_save.theta))
        result = e + (state - e_save);
    else
        result = {0, 0, NAN, NAN, NAN};
    
    plot.flush();
    return result;
}

void
autolabor::particle_filter_t::
initialize(const autolabor::odometry_t<> &state,
           const Eigen::Vector2d &measure) {
    measure_save = measure;
    match_save   = state;
    const auto step  = 2 * M_PI / states.size();
    auto       value = -M_PI;
    for (auto  &item : states)
        item = {0, 0, measure[0], measure[1], value += step};
}
