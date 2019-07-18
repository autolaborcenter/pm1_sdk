//
// Created by User on 2019/7/18.
//

#ifndef PM1_SDK_FUSION_LOCATOR_T_HPP
#define PM1_SDK_FUSION_LOCATOR_T_HPP

#include <deque>
#include <numeric>

#include "../telementry_t.h"
#include "matcher_t.hpp"
#include "transformer_t.hpp"

namespace autolabor {
    template<size_t> class fusion_locator_t {
        using location_pair = std::pair<telementry_t, telementry_t>;
        using stamped_data = stamped_t<telementry_t>;
        
        std::deque<location_pair> pairs;
        
        matcher_t <telementry_t, telementry_t> matcher;
        transformer_t<>                        transformer;
    
    public:
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
            using vector_t = typename decltype(transformer)::coordinates_t;
            
            auto location  = vector_t{pose.x, pose.y},
                 direction = vector_t{std::cos(pose.theta), std::sin(pose.theta)};
            transformer(location);
            transformer(direction);
            return {location[0], location[1], std::atan2(direction[1], direction[0])};
        }
    };
} // namespace autolabor

template<size_t max_size> struct types {
    using vector2d_t = Eigen::Vector<double, 2>;
    using linear_transformer_t = Eigen::Matrix<double, 2, 2>;
    using parameters_t =
    Eigen::Matrix<double, Eigen::Dynamic, 4, Eigen::StorageOptions::ColMajor,
                  2 * max_size, 4>;
    using targets_t = Eigen::Vector<double, 2 * max_size>;
};

template<size_t max_size>
void autolabor::fusion_locator_t<max_size>::refresh() {
    location_pair pair;
    while (matcher.match(pair.first, pair.second))
        pairs.push_back(pair);
    if (pairs.size() > max_size)
        pairs.erase(pairs.begin(), pairs.end() - max_size);
    
    using _t = types<max_size>;
    
    typename _t::linear_transformer_t a;
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
            auto centres = std::accumulate(
                pairs.begin(), pairs.end(), location_pair{},
                [](const location_pair &sum, const location_pair &item) {
                    return location_pair{
                        {sum.first.x + item.first.x,   sum.first.y + item.first.y},
                        {sum.second.x + item.second.x, sum.second.y + item.second.y}};
                });
            
            typename _t::vector2d_t cs{centres.first.x / size, centres.first.y / size},
                                    ct{centres.second.x / size, centres.second.y / size};
            
            typename _t::parameters_t p;
            typename _t::targets_t    y;
            p.resize(2 * size, 4);
            y.resize(2 * size);
            
            size_t          i = 0;
            for (const auto &item : pairs) {
                typename _t::vector2d_t source{item.first.x, item.first.y},
                                        target{item.second.x, item.second.y};
                source -= cs;
                target -= ct;
                
                p(i, 0) = source[0];
                p(i, 1) = source[1];
                p(i, 2) = 0;
                p(i, 3) = 0;
                y(i)    = target[0];
                ++i;
                
                p(i, 0) = 0;
                p(i, 1) = 0;
                p(i, 2) = source[0];
                p(i, 3) = source[1];
                y(i)    = target[1];
                ++i;
            }
            auto pt   = p.transpose();
            auto temp = pt * p;
            std::cout << temp.determinant() << std::endl;
        }
            break;
    }
    std::cout << transformer.core << std::endl;
}

#endif // PM1_SDK_FUSION_LOCATOR_T_HPP
