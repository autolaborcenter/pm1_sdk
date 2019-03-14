//
// Created by ydrml on 2019/3/14.
//

#include "chassis_t.h"

const float pi_f = 3.141592653589793238462643383279502884;

const struct chassis_t default_chassis = _chassis_of(
		+32000,
		-16384,
		0.4205,
		0.3170,
		0.2074,
		3 * 2 * pi_f,
		pi_f / 2);
