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
	float difference = fabsf(target->rudder - current->rudder);
	result.speed = difference > config->optimize_width
	               ? 0
	               : (1 - difference / config->optimize_width) * target->speed;
	// 在速度空间中限速
	if (result.speed != 0) {
		struct velocity temp = physical_to_velocity(&result, chassis);
		
		float ratio_v = config->max_v / fabsf(temp.v);
		float ratio_w = config->max_v / fabsf(temp.w);
		result.speed *= fminf(fminf(ratio_v, ratio_w), 1);
	}
	// 动态调速
	result.speed = result.speed > current->speed
	               ? fminf(current->speed + config->acceleration, result.speed)
	               : fmaxf(current->speed - config->acceleration, result.speed);
	return result;
}
