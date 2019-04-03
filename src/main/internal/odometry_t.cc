//
// Created by ydrml on 2019/3/12.
//

#include <cmath>
#include <tuple>
#include "odometry_t.hh"

using namespace autolabor;

void odometry_t::clear() {
	s = sa = x = y = theta = vx = vy = w = 0;
}

odometry_t odometry_t::operator+(const odometry_t &delta) const {
	auto sin = std::sin(theta);
	auto cos = std::cos(theta);
	
	return {s + delta.s,
	        sa + delta.sa,
	        x + delta.x * cos - delta.y * sin,
	        y + delta.x * sin + delta.y * cos,
	        theta + delta.theta,
	        delta.vx,
	        delta.vy,
	        delta.w};
}

odometry_t odometry_t::operator-(const odometry_t &mark) const {
	auto sin = std::sin(-mark.theta);
	auto cos = std::cos(-mark.theta);
	auto dx  = x - mark.x;
	auto dy  = y - mark.y;
	
	return {s - mark.s,
	        sa - mark.sa,
	        dx * cos - dy * sin,
	        dx * sin + dy * cos,
	        theta - mark.theta,
	        vx,
	        vy,
	        w};
}

delta_differential_t::operator odometry_t() {
	double ds = (left + rigth) / 2,
	       da = (rigth - left) / width,
	       dx,
	       dy,
	       dt = 1 / time.count();
	
	if (da == 0) {
		dx = left;
		dy = 0;
	} else {
		auto r = ds / da;
		dx = r * std::sin(da);
		dy = r * (1 - std::cos(da));
	}
	
	return {std::abs(ds),
	        std::abs(da),
	        dx,
	        dy,
	        da,
	        dt * dx,
	        dt * dy,
	        dt * da};
}
