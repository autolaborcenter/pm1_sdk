//
// Created by User on 2019/7/4.
//

#ifndef PM1_SDK_POINT_T_H
#define PM1_SDK_POINT_T_H


#include <cstddef>

namespace path_follower {
    /**
     * 点
     */
    struct point_t {
        double
            x         = 0,
            y         = 0;
        unsigned char
            tip_order = 255;
    };
}


#endif //PM1_SDK_POINT_T_H
