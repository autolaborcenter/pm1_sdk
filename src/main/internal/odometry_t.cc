//
// Created by ydrml on 2019/3/12.
//

#include <cmath>
#include <tuple>
#include "odometry_t.hh"

using namespace autolabor::pm1;

odometry_t odometry_t::operator+(const delta_odometry_t<> &delta) const {
	double ds      = (delta.left + delta.rigth) / 2,
	       dx, dy,
	       d_theta = (delta.rigth - delta.left) / width,
	       dt      = 1 / delta.time.count();
	
	if (d_theta == 0) {
		dx = delta.left;
		dy = 0;
	} else {
		auto r = (delta.rigth + delta.left) / 2 / d_theta;
		dx = r * std::sin(d_theta);
		dy = r * (1 - std::cos(d_theta));
	}
	
	auto sin = std::sin(theta);
	auto cos = std::cos(theta);
	auto _   = dx * cos - dy * sin;
	dy = dx * sin + dy * cos;
	dx = _;
	
	return {width,
	        s + ds,
	        x + dx,
	        y + dy,
	        theta + d_theta,
	        dt * dx,
	        dt * dy,
	        dt * d_theta};
}

void odometry_t::clear() {
	s = x = y = theta = vx = vy = w = 0;
}

odometry_t &odometry_t::operator=(const odometry_t &others) {
	if (width != others.width)
		throw std::exception("cannot assign odometry to different chassis");
	
	s     = others.s;
	x     = others.x;
	y     = others.y;
	theta = others.theta;
	vx    = others.vx;
	vy    = others.vy;
	w     = others.w;
	
	return *this;
}
