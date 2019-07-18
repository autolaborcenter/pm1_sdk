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
    class fusion_locator_t {
        using location_pair = std::pair<telementry_t, telementry_t>;
        using stamped_data  = stamped_t<telementry_t>;
        
        std::deque<location_pair> pairs;
        
        matcher_t <telementry_t, telementry_t> matcher;
        transformer_t<>                        transformer;
        
        size_t max_size;
    public:
        explicit fusion_locator_t(size_t max_size)
            : max_size(max_size) {}
        
        /** 向主配队列添加元素 */
        void push_back_master(const stamped_data &data) { matcher.push_back_master(data); }
        
        /** 向匹配队列增加元素 */
        void push_back_helper(const stamped_data &data) { matcher.push_back_helper(data); }
        
        /** 匹配 */
        void refresh() {
            location_pair pair;
            while (matcher.match(pair.first, pair.second))
                pairs.push_back(pair);
            if (pairs.size() > max_size)
                pairs.erase(pairs.begin(), pairs.end() - max_size);
            
            using vector_t = typename decltype(transformer)::coordinates_t;
            using linear_t = typename decltype(transformer)::linear_t;
            
            linear_t a;
            a << 1, 0, 0, 1;
            transformer.build({0, 0}, {0, 0}, a);
            
            //            const auto size = pairs.size();
            //            switch (size) {
            //                case 0:
            //                case 1:
            //                    return invalid_pose;
            //                case 2:
            //                    break;
            //                default: {
            //                    auto centres = std::accumulate(
            //                        pairs.begin(), pairs.end(),
            //                        location_pair{},
            //                        [](const location_pair &sum,
            //                           const location_pair &item) {
            //                            return location_pair{
            //                                {
            //                                    sum.first.x + item.first.x,
            //                                    sum.first.y + item.first.y
            //                                },
            //                                {
            //                                    sum.second.x + item.second.x,
            //                                    sum.second.y + item.second.y
            //                                }
            //                            };
            //                        });
            //                    centres.first.x /= size;
            //                    centres.first.y /= size;
            //                    centres.second.x /= size;
            //                    centres.second.y /= size;
            //                }
            //                    break;
            //            }
        }
        
        [[nodiscard]] pose_t operator[](pose_t pose) const {
            using vector_t = typename decltype(transformer)::coordinates_t;
            
            auto location  = vector_t{pose.x, pose.y},
                 direction = vector_t{std::cos(pose.theta), std::sin(pose.theta)};
            transformer(location);
            transformer(direction);
            return {location[0], location[1], std::atan2(direction[1], direction[0])};
        }
    };
}


#endif //PM1_SDK_FUSION_LOCATOR_T_HPP
