//
// Created by User on 2019/7/19.
//

#include "fusion_locator_t.hh"

#include <numeric>
#include <filesystem>
#include <eigen3/Eigen/LU>
#include <iostream>

autolabor::fusion_locator_t::fusion_locator_t(size_t queue_size)
    : queue_size(queue_size),
      plot("locator.txt", std::ios::out) {
    std::error_code _noexcept;
    std::filesystem::remove("locator.txt", _noexcept);
}

bool autolabor::fusion_locator_t::update_queue() {
    auto          result = false;
    location_pair pair;
    while (matcher.match(pair.first, pair.second)) {
        if (!pairs.empty() && (pair.second - pairs.back().second).norm() < 0.05)
            continue;
        result = true;
        pairs.push_back(pair);
        plot << pair.first[0] << ' ' << pair.first[1] << ' '
             << pair.second[0] << ' ' << pair.second[1] << std::endl;
    }
    return result;
}

autolabor::fusion_locator_t::~fusion_locator_t() {
    plot.flush();
    plot.close();
}

void autolabor::fusion_locator_t::push_back_master(const autolabor::fusion_locator_t::stamped_data &data) {
    matcher.push_back_master(data);
}

void autolabor::fusion_locator_t::push_back_helper(const autolabor::fusion_locator_t::stamped_data &data) {
    matcher.push_back_helper(data);
}

bool autolabor::fusion_locator_t::refresh() {
    //    if (!update_queue())
    //        return false;
    if (pairs.size() > queue_size)
        pairs.erase(pairs.begin(), pairs.end() - queue_size);
    
    const auto size = pairs.size();
    if (size < 3) return false;
    
    // 求质心
    auto centres = std::accumulate(
        pairs.begin(), pairs.end(),
        location_pair{Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()},
        [](const location_pair &sum, const location_pair &item) {
            return location_pair{sum.first + item.first, sum.second + item.second};
        });
    
    auto ct = centres.first / size,
         cs = centres.second / size;
    
    // 初始化
    Eigen::MatrixXd p;
    Eigen::VectorXd y;
    p.resize(2 * size, 4);
    y.resize(2 * size);
    
    size_t          i = 0;
    for (const auto &item : pairs) {
        auto target = item.first - ct,
             source = item.second - cs;
        
        p.row(i) << source[0], source[1], 0, 0;
        y(i++) = target[0];
        p.row(i) << 0, 0, source[0], source[1];
        y(i++) = target[1];
    }
    
    auto pt  = p.transpose();
    auto ptp = pt * p;
    if (std::abs(ptp.determinant()) < 1E-6) {
        std::cout << std::abs(ptp.determinant()) << std::endl;
        return false;
    }
    
    Eigen::Matrix2d a;
    
    auto solve = ptp.inverse() * pt * y;
    a << solve[0], solve[1], solve[2], solve[3];
    auto det = a.determinant();
    if (std::abs(det) < 0.8 || std::abs(det) > 1.25)
        return false;
    transformer.build(cs, ct, a);
    
    auto test = pairs.back().second;
    transformer(test);
    
    std::cout << "det = " << det << std::endl
              << a << std::endl
              << "source = " << pairs.back().second.transpose() << std::endl
              << "target = " << pairs.back().first.transpose() << std::endl
              << "transformed = " << test.transpose() << std::endl;
    return true;
}

autolabor::pose_t autolabor::fusion_locator_t::operator[](autolabor::pose_t pose) const {
    if (pairs.empty()) return pose;
    
    Eigen::Vector2d delta = Eigen::Vector2d{pose.x, pose.y} - pairs.back().second,
                    direction{std::cos(pose.theta), std::sin(pose.theta)};
    transformer(delta);
    transformer(direction);
    auto location = pairs.back().first + delta;
    return {location[0], location[1], std::atan2(direction[1], direction[0])};
}

void autolabor::fusion_locator_t::push_back_pair(
    const Eigen::Vector2d &target,
    const Eigen::Vector2d &source) {
    pairs.emplace_back(target, source);
}
