//
// Created by User on 2019/4/2.
//

#ifndef PM1_SDK_PROCESS_CONTROLLER_HH
#define PM1_SDK_PROCESS_CONTROLLER_HH

#include <algorithm>

extern "C" {
#include "control_model/chassis_config_t.h"
}

namespace autolabor {
    /** 过程参数 */
    struct process_t {
        double begin, // 起点
               end,   // 终点
               speed; // 目标速度
        
        double operator[](double current) const {
            return (current - begin) / (end - begin);
        }
    };
    
    /** 过程控制器 */
    struct process_controller {
        double speed_begin,  // 起步速度
               speed_end,    // 终止速度
               acceleration, // 加速度
               deceleration; // 减速度
        
        inline double operator()(const process_t &process,
                                 double current) const {
            auto abs       = std::abs(process.speed);
            auto optimized = std::min({abs,
                                       acceleration * std::max({.0, current - process.begin}) + speed_begin,
                                       deceleration * std::max({.0, process.end - current}) + speed_end});
            return process.speed > 0 ? +optimized : -optimized;
        }
    };
}


#endif //PM1_SDK_PROCESS_CONTROLLER_HH
