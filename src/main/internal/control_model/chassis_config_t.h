//
// Created by ydrml on 2019/3/14.
//

#ifndef PM1_SDK_CHASSIS_CONFIG_T_H
#define PM1_SDK_CHASSIS_CONFIG_T_H

#define PI_F 3.141592653589793238462643383279502884f

/**
 * 参数：
 * 轮间距
 * 轴间距
 * 轮直径
 * 最大轮速（圈/秒）
 */
#define CHASSIS_CONFIG(\
   WIDTH, \
   LENGTH, \
   DIAMETER, \
   WHEEL_SPEED \
) {                          \
 (WIDTH),                    \
 (LENGTH),                   \
 (DIAMETER) / 2,             \
 PI_F*(DIAMETER)*WHEEL_SPEED \
}

const float pi_f           = PI_F;

struct chassis_config_t {
	float width,
	      length,
	      radius,
	      max_wheel_speed;
} const     default_config = CHASSIS_CONFIG(0.41f,
                                            0.317f,
                                            0.20f,
                                            3);


#endif // PM1_SDK_CHASSIS_CONFIG_T_H
