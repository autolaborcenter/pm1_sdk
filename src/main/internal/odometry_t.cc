//
// Created by ydrml on 2019/3/12.
//

#include <cmath>
#include <tuple>
#include "odometry_t.hh"
#include "mechanical.h"

using namespace autolabor::pm1;

void odometry_t::operator+=(const odometry_update_info<> &info) {
	s += (info.d_left + info.d_rigth) / 2;
	
	double dx, dy, d_theta = (info.d_rigth - info.d_left) / mechanical::width;
	
	if (d_theta == 0) {
		dx = info.d_left;
		dy = 0;
	} else {
		auto r = (info.d_rigth + info.d_left) / 2 / d_theta;
		dx = r * std::sin(d_theta);
		dy = r * (1 - std::cos(d_theta));
	}
	
	auto sin = std::sin(theta);
	auto cos = std::cos(theta);
	auto _   = dx * cos - dy * sin;
	dy = dx * sin + dy * cos;
	dx = _;
	
	x += dx;
	y += dy;
	theta += d_theta;
	
	vx = dx / info.d_t.count();
	vy = dy / info.d_t.count();
	w  = d_theta / info.d_t.count();
}
