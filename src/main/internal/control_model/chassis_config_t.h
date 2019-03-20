//
// Created by ydrml on 2019/3/14.
//

#ifndef PM1_SDK_CHASSIS_CONFIG_T_H
#define PM1_SDK_CHASSIS_CONFIG_T_H

#include "pi.h"

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
   DIAMETER \
) {             \
 (WIDTH),       \
 (LENGTH),      \
 (DIAMETER) / 2 \
}

struct chassis_config_t {
	float width,
	      length,
	      radius;
};

extern const float                   pi_f;
extern const struct chassis_config_t default_config;

#endif // PM1_SDK_CHASSIS_CONFIG_T_H
