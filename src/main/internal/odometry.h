//
// Created by User on 2019/7/25.
//

#ifndef PM1_SDK_ODOMETRY_H
#define PM1_SDK_ODOMETRY_H


#include <utilities/odometry_t.hpp>
#include "control_model/chassis_config_t.h"

namespace autolabor {
    namespace pm1 {
        /**
         * 推算里程增量
         * @param left   左轮转角增量
         * @param right  右轮转角增量
         * @param config 底盘结构参数
         * @return 里程增量
         */
        odometry_t<odometry_type::delta> wheels_to_odometry(
            double left,
            double right,
            const chassis_config_t &config);
    } // namespace pm1
} // namespace autolabor


#endif //PM1_SDK_ODOMETRY_H
