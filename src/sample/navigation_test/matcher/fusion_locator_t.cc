//
// Created by User on 2019/7/19.
//

#include "fusion_locator_t.hh"

#include <numeric>
#include <filesystem>
#include <iostream>
#include <eigen3/Eigen/LU>

autolabor::fusion_locator_t::fusion_locator_t(
    size_t queue_size,
    double step
) : queue_size(queue_size),
    step(step),
    plot("locator.txt", std::ios::out) {
    if (step < 0.01) throw std::logic_error("step is too short.");
    std::error_code _noexcept;
    std::filesystem::remove("locator.txt", _noexcept);
}

bool autolabor::fusion_locator_t::update_queue() {
    auto            result = false;
    location_pair_t pair;
    while (matcher.match(pair.target, pair.source)) {
        if (!pairs.empty() && (pair.source - pairs.back().source).norm() < step)
            continue;
        result = true;
        pairs.push_back(pair);
        plot << pair.target[0] << ' ' << pair.target[1] << ' '
             << pair.source[0] << ' ' << pair.source[1] << std::endl;
    }
    return result;
}

autolabor::fusion_locator_t::~fusion_locator_t() {
    plot.flush();
    plot.close();
}

void autolabor::fusion_locator_t::push_back_master(
    const autolabor::fusion_locator_t::stamped_data &data) {
    matcher.push_back_master(data);
}

void autolabor::fusion_locator_t::push_back_helper(
    const autolabor::fusion_locator_t::stamped_data &data) {
    matcher.push_back_helper(data);
}

void autolabor::fusion_locator_t::push_back_pair(
    const Eigen::Vector2d &target,
    const Eigen::Vector2d &source) {
    pairs.push_back({target, source});
}

void autolabor::fusion_locator_t::refresh() {
    if (!update_queue()) {
        state = false;
        return;
    }
    if (pairs.size() > queue_size)
        pairs.erase(pairs.begin(), pairs.end() - queue_size);
    
    const auto size = pairs.size();
    if (size < 3) {
        state = false;
        return;
    }
    
    // 求质心
    auto centres = std::accumulate(
        pairs.begin(), pairs.end(),
        location_pair_t{Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()},
        [](const location_pair_t &sum, const location_pair_t &item) {
            return location_pair_t{sum.target + item.target, sum.source + item.source};
        });
    
    auto ct = pairs.back().target,
         cs = pairs.back().source;
    
    // 初始化
    Eigen::MatrixXd p;
    Eigen::VectorXd y;
    p.resize(2 * size, 4);
    y.resize(2 * size);
    
    size_t          i = 0;
    for (const auto &item : pairs) {
        auto target = item.target - ct,
             source = item.source - cs;
        
        p.row(i) << source[0], source[1], 0, 0;
        y(i++) = target[0];
        p.row(i) << 0, 0, source[0], source[1];
        y(i++) = target[1];
    }
    
    auto pt  = p.transpose();
    auto ptp = pt * p;
    if (std::abs(ptp(0, 0) * ptp(1, 1) - ptp(1, 0) * ptp(0, 1)) < 1E-6) {
        state = false;
        return;
    }
    
    Eigen::Matrix2d a;
    
    auto solve = ptp.inverse() * pt * y;
    a << solve[0], solve[1], solve[2], solve[3];
    auto det = a(0, 0) * a(1, 1) - a(1, 0) * a(0, 1);
    auto dot = a(0, 0) * a(0, 1) + a(1, 0) * a(1, 1);
    
    //    std::cout << "det = " << det << std::endl
    //              << "dot = " << dot << std::endl;
    
    //    std::cout << "det = " << det << std::endl
    //              << "------------------------" << std::endl
    //              << a << std::endl
    //              << "------------------------" << std::endl;
    
    if ((state = (0.25 < std::abs(det) && std::abs(det) < 4.0) && std::abs(dot) < 2))
        transformer.build(cs, ct, a);
    
    if (state)
        std::cout << solve[0] << ' '
                  << solve[2] << ' '
                  << solve[1] << ' '
                  << solve[3] << std::endl;
    
    //    std::cout << "x0 y0 x1 y1" << std::endl;
    //    for (const auto &pair : pairs)
    //        std::cout << pair.target.transpose() << ' '
    //                  << transformer(pair.source).transpose() << std::endl;
}

autolabor::pose_t autolabor::fusion_locator_t::operator[](autolabor::pose_t pose) const {
    if (pairs.empty()) return pose;
    Eigen::Vector2d
        location  = transformer(Eigen::Vector2d{pose.x, pose.y}),
        direction = transformer({std::cos(pose.theta), std::sin(pose.theta)});
    return {location[0], location[1], std::atan2(direction[1], direction[0])};
}

bool autolabor::fusion_locator_t::get_state() {
    return state;
}
