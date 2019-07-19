//
// Created by User on 2019/7/19.
//

#include "fusion_locator_t.hh"

#include <numeric>
#include <filesystem>
#include <eigen3/Eigen/LU>

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
    if (!update_queue())
        return false;
    if (pairs.size() > queue_size)
        pairs.erase(pairs.begin(), pairs.end() - queue_size);
    
    Eigen::Matrix2d a;
    a << 1, 0, 0, 1;
    const auto size = pairs.size();
    switch (size) {
        case 0:  // 没有匹配点对
            transformer.build({0, 0}, {0, 0}, a);
            return false;
        case 1:  // 一点匹配，可平移
            transformer.build(pairs.front().second, pairs.front().first, a);
            return false;
        case 2: { // 两点匹配，可旋转
            auto source = pairs[1].second - pairs[0].second,
                 target = pairs[1].first - pairs[0].first;
            auto angle  = std::atan2(target[1], target[0]) - std::atan2(source[1], source[0]),
                 cos    = std::cos(angle),
                 sin    = std::sin(angle);
            a << cos, -sin, sin, cos;
            transformer.build(source, target, a);
            return false;
        }
        default: { // 更多点匹配，确定坐标系
            // 求质心
            auto centres = std::accumulate(
                pairs.begin(), pairs.end(), location_pair{},
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
            if (std::abs(ptp.determinant()) < 1E-6)
                return false;
            
            auto solve = ptp.inverse() * pt * y;
            a << solve[0], solve[1], solve[2], solve[3];
            auto det = a.determinant();
            if (std::abs(det) < 0.625 || std::abs(det) > 1.6)
                return false;
            transformer.build(cs, ct, a);
            return true;
        }
    }
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
