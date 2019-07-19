//
// Created by User on 2019/7/18.
//

#ifndef PM1_SDK_FUSION_LOCATOR_T_HPP
#define PM1_SDK_FUSION_LOCATOR_T_HPP

#include <deque>
#include <numeric>
#include <filesystem>
#include <fstream>
#include <eigen3/Eigen/LU>

#include "../telementry_t.h"
#include "matcher_t.hpp"
#include "transformer_t.hpp"

namespace autolabor {
    /**
     * 融合定位器
     */
    class fusion_locator_t {
        using location_pair = std::pair<telementry_t, telementry_t>;
        using stamped_data = stamped_t<telementry_t>;
        
        std::deque<location_pair> pairs;
        
        matcher_t <telementry_t, telementry_t> matcher;
        transformer_t<>                        transformer;
    
        size_t queue_size;
        
        std::ofstream plot;
        
        // 更新队列
        inline bool update_queue() {
            auto          result = false;
            location_pair pair;
            while (matcher.match(pair.first, pair.second)) {
                if (!pairs.empty()) {
                    auto temp = pairs.back().second;
                    auto dx   = pair.second.x - temp.x,
                         dy   = pair.second.y - temp.y;
                    if (dx * dx + dy * dy < 0.05 * 0.05)
                        continue;
                }
                result = true;
                pairs.push_back(pair);
                plot << pair.first.x << ' ' << pair.first.y << ' '
                     << pair.second.x << ' ' << pair.second.y << std::endl;
            }
            return result;
        }
    
    public:
        explicit fusion_locator_t(size_t queue_size)
            : queue_size(queue_size),
              plot("log.txt", std::ios::out) {
            std::error_code _noexcept;
            std::filesystem::remove("log.txt", _noexcept);
        }
    
        ~fusion_locator_t() {
            plot.flush();
            plot.close();
        }
        
        /** 向主配队列添加元素 */
        void push_back_master(const stamped_data &data) {
            matcher.push_back_master(data);
        }
        
        /** 向匹配队列增加元素 */
        void push_back_helper(const stamped_data &data) {
            matcher.push_back_helper(data);
        }
        
        /** 匹配 */
        void refresh();
        
        [[nodiscard]] pose_t operator[](pose_t pose) const {
            Eigen::Vector2d location{pose.x, pose.y},
                            direction{std::cos(pose.theta), std::sin(pose.theta)};
            transformer(location);
            transformer(direction);
            return {location[0], location[1], std::atan2(direction[1], direction[0])};
        }
    };
} // namespace autolabor

void autolabor::fusion_locator_t::refresh() {
    if (!update_queue())
        return;
    if (pairs.size() > queue_size)
        pairs.erase(pairs.begin(), pairs.end() - queue_size);
    
    Eigen::Matrix2d a;
    a << 1, 0, 0, 1;
    const auto size = pairs.size();
    switch (size) {
        case 0: { // 没有匹配点对
            transformer.build({0, 0}, {0, 0}, a);
        }
            break;
        case 1: { // 一点匹配，可平移
            telementry_t t0{}, t1{};
            std::tie(t0, t1) = pairs.front();
            transformer.build({t0.x, t0.y}, {t1.x, t1.y}, a);
        }
            break;
        case 2: { // 两点匹配，可旋转
            telementry_t t[4]{};
            std::tie(t[0], t[1]) = pairs[0];
            std::tie(t[2], t[3]) = pairs[1];
            auto angle = std::atan2(t[3].y - t[1].y, t[3].x - t[1].x) -
                         std::atan2(t[2].y - t[0].y, t[2].x - t[0].x),
                 cos   = std::cos(angle), sin = std::sin(angle);
            a << cos, -sin, sin, cos;
            transformer.build({t[0].x, t[0].y}, {t[1].x, t[1].y}, a);
        }
            break;
        default: { // 更多点匹配，确定坐标系
            // 求质心
            auto centres = std::accumulate(
                pairs.begin(), pairs.end(), location_pair{},
                [](const location_pair &sum, const location_pair &item) {
                    return location_pair{
                        {sum.first.x + item.first.x,   sum.first.y + item.first.y},
                        {sum.second.x + item.second.x, sum.second.y + item.second.y}};
                });
    
            Eigen::Vector2d cs{centres.first.x / size, centres.first.y / size},
                            ct{centres.second.x / size, centres.second.y / size};
    
            std::cout << "---------------------------------------" << std::endl
                      << "cs = " << cs.transpose() << std::endl
                      << "ct = " << ct.transpose() << std::endl;
            
            // 初始化
            Eigen::MatrixXd p;
            Eigen::VectorXd y;
            p.resize(2 * size, 4);
            y.resize(2 * size);
            
            size_t          i = 0;
            for (const auto &item : pairs) {
                auto source = Eigen::Vector2d{item.first.x, item.first.y} - cs,
                     target = Eigen::Vector2d{item.second.x, item.second.y} - ct;
                
                p.row(i) << source[0], source[1], 0, 0;
                y(i++) = target[0];
                p.row(i) << 0, 0, source[0], source[1];
                y(i++) = target[1];
            }
            auto pt  = p.transpose();
            auto ptp = pt * p;
            if (std::abs(ptp.determinant()) < 1E-6)
                break;
    
            auto solve = ptp.inverse() * pt * y;
            a << solve[0], solve[1], solve[2], solve[3];
            auto det = a.determinant();
            std::cout << "size = " << size << std::endl
                      << "det = " << det << std::endl;
            if (std::abs(det) < 0.625 || std::abs(det) > 1.6)
                break;
            std::cout << a << std::endl;
            transformer.build(cs, ct, a);
            auto temp  = pairs.back();
            auto value = Eigen::Vector2d{temp.second.x, temp.second.y};
            std::cout << "source = " << value << std::endl;
            transformer(value);
            std::cout << "target = " << value << std::endl;
        }
            break;
    }
}

#endif // PM1_SDK_FUSION_LOCATOR_T_HPP
