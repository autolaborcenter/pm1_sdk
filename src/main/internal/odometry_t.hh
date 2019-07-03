//
// Created by ydrml on 2019/3/12.
//

#ifndef PM1_SDK_ODOMETRY_T_HH
#define PM1_SDK_ODOMETRY_T_HH


#include <chrono>

namespace autolabor {
    /** 里程计信息 */
    struct odometry_t {
        double s, sa, x, y, theta, vx, vy, w;
        
        odometry_t operator+(const odometry_t &) const;
        
        odometry_t operator-(const odometry_t &) const;
        
        inline void operator+=(const odometry_t &delta) { *this = *this + delta; }
        
        inline void operator-=(const odometry_t &delta) { *this = *this - delta; }
        
        /** 清空里程 */
        void clear();
    };
    
    /** 差动里程计增量 */
    struct delta_differential_t {
        double width, left, right;
        std::chrono::duration<double, std::ratio<1>>
               interval;
    
        operator odometry_t(); // NOLINT(google-explicit-constructor,hicpp-explicit-conversions)
    };
}


#endif //PM1_SDK_ODOMETRY_T_HH
