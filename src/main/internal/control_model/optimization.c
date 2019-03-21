//
// Created by ydrml on 2019/3/20.
//

#include <math.h>
#include "optimization.h"

struct physical optimize(const struct physical *target,
                         const struct physical *current,
                         const struct chassis_config_t *chassis,
                         const struct optimize_config_t *config) {
	struct physical result = {0, current->rudder};
	// 等待后轮转动
	if (!isnan(target->rudder))
		result.speed = target->speed * fmaxf(0, 1 - fabsf(target->rudder - current->rudder) / config->optimize_width);
	
	// 在速度空间中限速
	struct velocity temp = physical_to_velocity(&result, chassis);
	result.speed *= fminf(1, fminf(config->max_v / fabsf(temp.v),
	                               config->max_w / fabsf(temp.w)));
	
	// 动态调速
	result.speed = result.speed > current->speed
	               ? fminf(current->speed + config->acceleration, result.speed)
	               : fmaxf(current->speed - config->acceleration, result.speed);
	return result;
}
