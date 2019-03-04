//
// Created by ydrml on 2019/2/25.
//

#include "chassis.hh"

#include <algorithm>
#include "time_extensions.h"
#include "can/can_define.h"
#include "can/parser.hh"
#include "mechanical.h"

using namespace autolabor::pm1;

using ecu0_speed    = ecu<0>::current_speed_rx;
using ecu0_position = ecu<0>::current_position_rx;
using ecu0_target   = ecu<0>::target_speed;

using ecu1_speed    = ecu<1>::current_speed_rx;
using ecu1_position = ecu<1>::current_position_rx;
using ecu1_target   = ecu<1>::target_speed;

using tcu0_speed    = tcu<0>::current_speed_rx;
using tcu0_position = tcu<0>::current_position_rx;
using tcu0_target   = tcu<0>::target_position;

using serial_ref = const std::shared_ptr<serial::Serial> &;

/** 发送数据包 */
template<class t>
inline void write(serial_ref port, const msg_union<t> &msg) {
	port->write(msg.bytes, sizeof(t));
}

/**
 * 获取存储区中大端存储的第一个数据
 *
 * @tparam t    数据类型
 * @param bytes 存储区指针起点
 * @return      数据
 */
template<class t>
inline t get_first(const uint8_t *bytes) {
	msg_union<t> temp{};
	std::reverse_copy(bytes, bytes + sizeof(t), temp.bytes);
	return temp.data;
}

std::tuple<double, double, double>
calculate_odometry(double delta_left, double delta_right) {
	const auto theta = (delta_right - delta_left) / mechanical::width;
	double     x, y;
	if (theta == 0) {
		x = delta_left;
		y = 0;
	} else {
		const auto r = (delta_left + delta_right) / 2 / theta;
		const auto d = 2 * r * std::sin(theta / 2);
		x = 2 * d * std::sin(theta / 2);
		y = 2 * d * std::cos(theta / 2);
	}
	return {x, y, theta};
}

void rotate(double &x, double &y, double theta) {
	double _;
	auto   sin = std::sin(theta);
	auto   cos = std::cos(theta);
	_ = x * cos - y * sin;
	y = x * sin + y * cos;
	x = _;
}

chassis::chassis(const std::string &port_name)
		: port(new serial::Serial(port_name, 115200,
		                          serial::Timeout(serial::Timeout::max(), 5, 0, 0, 0))),
		  received(false) {
	auto port_ptr = port;
	
	// 设置超时时间：200 ms
	write(port_ptr, pack<ecu<>::timeout>({2, 0}));
	
	// 定时询问前轮
	std::thread([port_ptr] {
		auto time = mechdancer::common::now();
		while (port_ptr->isOpen()) {
			time += std::chrono::milliseconds(100);
			try {
				write(port_ptr, pack<ecu<>::current_position_tx>());
			} catch (std::exception &e) {}
			std::this_thread::sleep_until(time);
		}
	}).detach();
	
	// 定时询问后轮
	std::thread([port_ptr] {
		auto time = mechdancer::common::now();
		while (port_ptr->isOpen()) {
			time += std::chrono::milliseconds(100);
			try {
				write(port_ptr, pack<tcu<0>::current_position_tx>());
			} catch (std::exception &e) {}
			std::this_thread::sleep_until(time);
		}
	}).detach();
	
	// 接收
	std::thread([port_ptr, this] {
		std::string buffer;
		parser      parser;
		auto        left_ready  = false,
		            right_ready = false;
		auto        delta_left  = .0,
		            delta_right = .0;
		
		while (port_ptr->isOpen()) {
			// 接收
			try { buffer = port_ptr->read(); }
			catch (std::exception &) { buffer = ""; }
			
			if (buffer.empty()) continue;
			
			// 解析
			auto result = parser(*buffer.begin());
			if (result.type != parser::result_type::message)
				continue;
			
			// 处理
			const auto msg   = result.message;
			const auto bytes = msg.data.data;
			
			if (ecu0_position::match(msg)) {
				auto value = get_first<int>(bytes) * mechanical::wheel_k;
				delta_left = (value - _left) * mechanical::radius;
				_left      = value;
				if (right_ready) {
					right_ready = false;
					double x, y, theta;
					std::tie(x, y, theta) = calculate_odometry(delta_left, delta_right);
					rotate(x, y, odometry.theta);
					odometry.x += x;
					odometry.y += y;
					odometry.theta += theta;
					std::cout << odometry.x << '\t'
					          << odometry.y << '\t'
					          << odometry.theta << std::endl;
				} else
					left_ready = true;
			} else if (ecu1_position::match(msg)) {
				auto value = get_first<int>(bytes) * mechanical::wheel_k;
				delta_right = (value - _right) * mechanical::radius;
				_right      = value;
				if (left_ready) {
					left_ready = false;
					double x, y, theta;
					std::tie(x, y, theta) = calculate_odometry(delta_left, delta_right);
					rotate(x, y, odometry.theta);
					odometry.x += x;
					odometry.y += y;
					odometry.theta += theta;
					std::cout << odometry.x << '\t'
					          << odometry.y << '\t'
					          << odometry.theta << std::endl;
				} else
					right_ready = true;
			} else if (tcu0_position::match(msg)) {
				_rudder = get_first<short>(bytes) * mechanical::rudder_k;
				
				write(port_ptr, pack<ecu0_target>({target_left.bytes[3],
				                                   target_left.bytes[2],
				                                   target_left.bytes[1],
				                                   target_left.bytes[0]}));
				write(port_ptr, pack<ecu1_target>({target_right.bytes[3],
				                                   target_right.bytes[2],
				                                   target_right.bytes[1],
				                                   target_right.bytes[0]}));
				write(port_ptr, pack<tcu0_target>({target_rudder.bytes[1],
				                                   target_rudder.bytes[0]}));
			}
		}
	}).detach();
}

chassis::~chassis() {
	port->close();
}

double chassis::left() const {
	return _left;
}

double chassis::right() const {
	return _right;
}

double chassis::rudder() const {
	return _rudder;
}

void chassis::left(double target) const {
	target_left.data = static_cast<int> (target / mechanical::wheel_k);
}

void chassis::right(double target) const {
	target_right.data = static_cast<int> (target / mechanical::wheel_k);
}

void chassis::rudder(double target) const {
	target_rudder.data = static_cast<short> (target / mechanical::rudder_k);
}
