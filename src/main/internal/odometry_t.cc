//
// Created by ydrml on 2019/3/12.
//

#include <cmath>
#include <tuple>
#include "odometry_t.hh"
#include "mechanical.h"

using namespace autolabor::pm1;

struct delta {
	const double x, y, theta;
	
	inline delta rotate(double angle);
};

/**
 * 计算机器人坐标系下的里程计
 *
 * @param delta_left  左轮变化量
 * @param delta_right 右轮变化量
 * @param theta       车身转角
 * @param x           横坐标相对变化
 * @param y           纵坐标相对变化
 */
inline delta calculate_odometry(double delta_left, double delta_right) {
	
	double x, y, theta = (delta_right - delta_left) / mechanical::width;
	
	if (theta == 0) {
		x = delta_left;
		y = 0;
	} else {
		const auto sin = std::sin(theta / 2);
		const auto cos = std::cos(theta / 2);
		const auto r   = (delta_left + delta_right) / 2 / theta;
		const auto d   = 2 * r * sin;
		x = d * cos;
		y = d * sin;
	}
	
	return {x, y, theta};
}

void odometry_t::operator+=(const odometry_update_info<> &info) {
	s += (info.d_left + info.d_rigth) / 2;
	
	auto delta = calculate_odometry(info.d_left, info.d_rigth).rotate(theta);
	
	x += delta.x;
	y += delta.y;
	theta += delta.theta;
	
	vx = delta.x / info.d_t.count();
	vy = delta.y / info.d_t.count();
	w  = delta.theta / info.d_t.count();
}

delta delta::rotate(double angle) {
	auto sin = std::sin(angle);
	auto cos = std::cos(angle);
	return {x * cos - y * sin,
	        x * sin + y * cos,
	        theta};
}
