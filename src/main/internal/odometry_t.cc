//
// Created by ydrml on 2019/3/12.
//

#include <cmath>
#include <tuple>
#include "odometry_t.hh"

using namespace autolabor;

void odometry_t::clear() {
	s = x = y = theta = vx = vy = w = 0;
}

odometry_t odometry_t::operator+(const odometry_t &delta) const {
	auto sin = std::sin(theta);
	auto cos = std::cos(theta);
	
	return {s + delta.s,
	        x + delta.x * cos - delta.y * sin,
	        y + delta.x * sin + delta.y * cos,
	        theta + delta.theta,
	        delta.vx,
	        delta.vy,
	        delta.w};
}

odometry_t odometry_t::operator-(const odometry_t &absolute) const {
	auto sin = std::sin(-theta);
	auto cos = std::cos(-theta);
	
	return {s - absolute.s,
	        (x - absolute.x) * cos - (y - absolute.y) * sin,
	        (x - absolute.x) * sin + (y - absolute.y) * cos,
	        theta - absolute.theta,
	        vx,
	        vy,
	        w};
}

delta_differential_t::operator odometry_t() {
	double ds      = (left + rigth) / 2,
	       dx, dy,
	       d_theta = (rigth - left) / width,
	       dt      = 1 / time.count();
	
	if (d_theta == 0) {
		dx = left;
		dy = 0;
	} else {
		auto r = (rigth + left) / 2 / d_theta;
		dx = r * std::sin(d_theta);
		dy = r * (1 - std::cos(d_theta));
	}
	
	return {ds,
	        dx,
	        dy,
	        d_theta,
	        dt * dx,
	        dt * dy,
	        dt * d_theta};
}
