//
// Created by User on 2019/7/25.
//

#ifndef PM1_SDK_PM1_SDK_DEFINITIONS_H
#define PM1_SDK_PM1_SDK_DEFINITIONS_H


namespace autolabor {
    namespace pm1 {
        /**
         * 用于访问底盘参数的标识符
         */
        enum class parameter_id {
            width,           // 宽度（轮间距）
            length,          // 长度（轴间距）
            left_radius,     // 左轮半径
            right_radius,    // 右轮半径
            max_wheel_speed, // 最大动力轮角速度
            max_v,           // 最大底盘线速度
            max_w,           // 最大底盘角速度
            optimize_width,  // 优化函数半宽度
            acceleration,    // 最大动力轮角加速度
        };
        
        /**
         * 表示底盘状态的标识符
         */
        enum class chassis_state : unsigned char {
            offline  = 0x00, // 离线
            unlocked = 0x01, // 未锁定
            error    = 0x7f, // 已连接但异常
            locked   = 0xff  // 已锁定
        };
    } // namespace pm1
} // namespace autolabor


#endif //PM1_SDK_PM1_SDK_DEFINITIONS_H
