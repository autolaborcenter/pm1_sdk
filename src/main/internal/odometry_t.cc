//
// Created by ydrml on 2019/3/12.
//

#include <cmath>
#include <tuple>
#include "odometry_t.hh"

void autolabor::odometry_t::clear() {
	s = x = y = theta = vx = vy = w = 0;
}

autolabor::odometry_t operator+(const autolabor::odometry_t &odometry,
                                const autolabor::delta_differential_t<> &delta) {
	double ds      = (delta.left + delta.rigth) / 2,
	       dx, dy,
	       d_theta = (delta.rigth - delta.left) / delta.width,
	       dt      = 1 / delta.time.count();
	
	if (d_theta == 0) {
		dx = delta.left;
		dy = 0;
	} else {
		auto r = (delta.rigth + delta.left) / 2 / d_theta;
		dx = r * std::sin(d_theta);
		dy = r * (1 - std::cos(d_theta));
	}
	
	auto sin = std::sin(odometry.theta);
	auto cos = std::cos(odometry.theta);
	auto _   = dx * cos - dy * sin;
	dy = dx * sin + dy * cos;
	dx = _;
	
	return {odometry.s + ds,
	        odometry.x + dx,
	        odometry.y + dy,
	        odometry.theta + d_theta,
	        dt * dx,
	        dt * dy,
	        dt * d_theta};
}
