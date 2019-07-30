//
// Created by User on 2019/7/18.
//

#ifndef PM1_SDK_FUSION_LOCATOR_T_HH
#define PM1_SDK_FUSION_LOCATOR_T_HH

#include <deque>
#include <fstream>
#include <utilities/time_extensions.h>

#include "../pose_t.h"
#include "matcher_t.hpp"
#include "transformer_t.hpp"

namespace autolabor {
    /**
     * 融合定位器
     */
    class fusion_locator_t {
        struct location_pair_t {
            Eigen::Vector2d target, source;
        };
        using stamped_data = stamped_t<Eigen::Vector2d>;
    
        std::deque<location_pair_t> pairs;
        
        matcher_t<Eigen::Vector2d, Eigen::Vector2d> matcher;
        transformer_t<>                             transformer;
    
        // 匹配参数
        size_t queue_size;
        double step;
        
        std::ofstream plot;
    
        // 匹配状态
        bool state = false;
        
        // 更新队列
        bool update_queue();
    
    public:
        explicit fusion_locator_t(size_t queue_size, double step);
        
        ~fusion_locator_t();
    
        /** 向主配队列添加元素 */
        void push_back_master(const stamped_data &data);
        
        /** 向匹配队列增加元素 */
        void push_back_helper(const stamped_data &data);
    
        /** 向匹配队列增加匹配对 */
        void push_back_pair(const Eigen::Vector2d &target, const Eigen::Vector2d &source);
        
        /** 匹配 */
        void refresh();
    
        bool get_state();
        
        /** 变换一个坐标 */
        [[nodiscard]] pose_t operator[](pose_t pose) const;
    };
} // namespace autolabor

#endif // PM1_SDK_FUSION_LOCATOR_T_HH
