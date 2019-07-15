//
// Created by User on 2019/7/4.
//

#ifndef PM1_SDK_POINT_T_H
#define PM1_SDK_POINT_T_H


namespace path_follower {
    /**
     * 点类型
     */
    enum class point_type_t : unsigned char {
        general, // 一般
        tip      // 尖端
    };
    
    /**
     * 点
     */
    struct point_t {
        double
            x    = 0,
            y    = 0;
        point_type_t
            type = point_type_t::general;
    };
}


#endif //PM1_SDK_POINT_T_H
