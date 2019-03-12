//
// Created by ydrml on 2019/3/12.
//

#include <cmath>
#include "odometry_t.hh"
#include "mechanical.h"

using namespace autolabor::pm1;

/**
 * 计算机器人坐标系下的里程计
 *
 * @param delta_left  左轮变化量
 * @param delta_right 右轮变化量
 * @param theta       车身转角
 * @param x           横坐标相对变化
 * @param y           纵坐标相对变化
 */
inline void calculate_odometry(
		double delta_left,
		double delta_right,
		double &theta,
		double &x,
		double &y) {
	theta = (delta_right - delta_left) / mechanical::width;
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
}

/**
 * 坐标系旋转
 *
 * @param x     横坐标
 * @param y     纵坐标
 * @param theta 旋转弧度
 */
inline void rotate(double &x,
                   double &y,
                   double theta) {
	double _;
	auto   sin = std::sin(theta);
	auto   cos = std::cos(theta);
	_ = x * cos - y * sin;
	y = x * sin + y * cos;
	x = _;
}

void odometry_t::operator+=(const odometry_update_info<> &info) {
	s += (info.d_left + info.d_rigth) / 2;
	
	double _theta, _x, _y;
	calculate_odometry(info.d_left, info.d_rigth, _theta, _x, _y);
	rotate(x, y, theta);
	
	x += x;
	y += y;
	theta += theta;
	
	vx = x / info.d_t.count();
	vy = y / info.d_t.count();
	w  = theta / info.d_t.count();
}
