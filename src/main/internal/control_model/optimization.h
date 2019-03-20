//
// Created by ydrml on 2019/3/20.
//

#ifndef PM1_SDK_OPTIMIZATION_H
#define PM1_SDK_OPTIMIZATION_H

#include "model.h"

struct optimize_config_t {
	float optimize_width, // 优化宽度
	      acceleration,   // 加速度
	      max_v,          // 最大线速度
	      max_w;          // 最大角速度
};

struct physical optimize(const struct physical *target,
                         const struct physical *current,
                         const struct chassis_config_t *,
                         const struct optimize_config_t *);

#endif //PM1_SDK_OPTIMIZATION_H
