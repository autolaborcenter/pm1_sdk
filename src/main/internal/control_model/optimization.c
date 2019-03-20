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
	
	float difference = fabsf(target->rudder - current->rudder);
	result.speed = difference > config->optimize_width
	               ? 0
	               : (1 - difference / config->optimize_width) * target->speed;
	if (result.speed != 0) {
		struct velocity temp = physical_to_velocity(&result, chassis);
		
		float ratio_v = fabsf(temp.v) / config->max_v;
		float ratio_w = fabsf(temp.w) / config->max_w;
		result.speed *= fminf(fminf(ratio_v, ratio_w), 1);
	}
	result.speed = result.speed > current->speed
	               ? fminf(current->speed + config->acceleration, result.speed)
	               : fmaxf(current->speed - config->acceleration, result.speed);
	return result;
}
