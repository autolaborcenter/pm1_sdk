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

/** 发送数据包 */
template<class t>
inline const std::shared_ptr<serial::Serial> &operator<<(
		const std::shared_ptr<serial::Serial> &,
		const msg_union<t> &);

/** 获取存储区中大端存储的第一个数据 */
template<class t>
inline t get_first(const uint8_t *);

template<class pack_info_t, class data_t>
inline auto pack_into(const msg_union<data_t> &value) -> decltype(pack<pack_info_t>());

/** 里程计更新信息 */
template<class time_unit = std::chrono::duration<double, std::ratio<1>>>
struct odometry_update_info { double d_left, d_rigth; time_unit d_t; };

/** 更新轮速里程计 */
inline void operator+=(odometry_t &, odometry_update_info<>);

inline double optimize(double rho, double theta, double current_theta) {
	constexpr static auto limit = mechanical::pi / 3;
	
	auto difference = std::abs(theta - current_theta);
	return difference > limit ? 0
	                          : (1 - difference / limit) * rho;
}

chassis::chassis(const std::string &port_name)
		: port(new serial::Serial(port_name, 115200,
		                          serial::Timeout(serial::Timeout::max(), 5, 0, 0, 0))) {
	
	port << pack<ecu<>::timeout>({2, 0}) // 设置超时时间：200 ms
	     << pack<ecu<>::clear>();        // 底层编码器清零
	
	auto port_ptr = port;
	// 定时询问前轮
	std::thread([port_ptr] {
		auto time = mechdancer::common::now();
		while (port_ptr->isOpen()) {
			time += std::chrono::milliseconds(100);
			try {
				port_ptr << pack<ecu<>::current_position_tx>();
			} catch (std::exception &) {}
			std::this_thread::sleep_until(time);
		}
	}).detach();
	
	// 定时询问后轮
	std::thread([port_ptr] {
		auto time = mechdancer::common::now();
		while (port_ptr->isOpen()) {
			time += std::chrono::milliseconds(100);
			try {
				port_ptr << pack<tcu<0>::current_position_tx>();
			} catch (std::exception &) {}
			std::this_thread::sleep_until(time);
		}
	}).detach();
	
	// 接收
	std::thread([port_ptr, this] {
		std::string buffer;
		parser      parser;
		
		auto left_ready  = false,
		     right_ready = false;
		auto delta_left  = .0,
		     delta_right = .0;
		auto time        = mechdancer::common::now();
		
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
			
			if (ecu<0>::current_position_rx::match(msg)) {
				
				auto value = get_first<int>(bytes) * mechanical::wheel_k;
				delta_left = (value - _left) * mechanical::radius;
				_left      = value;
				if (right_ready) {
					std::lock_guard<std::mutex> _(lock);
					right_ready = false;
					
					auto now = mechdancer::common::now();
					_odometry += {delta_left, delta_right, now - time};
					time     = now;
				} else
					left_ready = true;
				
			} else if (ecu<1>::current_position_rx::match(msg)) {
				
				auto value = get_first<int>(bytes) * mechanical::wheel_k;
				delta_right = (value - _right) * mechanical::radius;
				_right      = value;
				if (left_ready) {
					std::lock_guard<std::mutex> _(lock);
					left_ready = false;
					
					auto now = mechdancer::common::now();
					_odometry += {delta_left, delta_right, now - time};
					time     = now;
				} else
					right_ready = true;
				
			} else if (tcu<0>::current_position_rx::match(msg)) {
				
				_rudder = get_first<short>(bytes) * mechanical::rudder_k;
				
				auto target    = mechanical::state::from_wheels(target_left, target_right);
				auto optimized = mechanical::state(optimize(target.rho, target.theta, _rudder), _rudder);
				
				msg_union<int>   left{}, right{};
				msg_union<short> temp{};
				left.data  = static_cast<int> (optimized.left / mechanical::wheel_k);
				right.data = static_cast<int> (optimized.right / mechanical::wheel_k);
				temp.data  = static_cast<short> (target_rudder / mechanical::rudder_k);
				port_ptr << pack_into<ecu<0>::target_speed, int>(left)
				         << pack_into<ecu<1>::target_speed, int>(right)
				         << pack_into<tcu<0>::target_position, short>(temp);
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
	target_left = target;
}

void chassis::right(double target) const {
	target_right = target;
}

void chassis::rudder(double target) const {
	target_rudder = target;
}

odometry_t chassis::odometry() const {
	std::lock_guard<std::mutex> _(lock);
	return _odometry;
}

void chassis::set(double v, double w) {
	auto target = mechanical::state::from_target(v, w);
	target_left   = target.left;
	target_right  = target.right;
	target_rudder = target.theta;
}

template<class t>
inline const std::shared_ptr<serial::Serial> &operator<<(
		const std::shared_ptr<serial::Serial> &port,
		const msg_union<t> &msg) {
	port->write(msg.bytes, sizeof(t));
	return port;
}

template<class t>
inline t get_first(const uint8_t *bytes) {
	msg_union<t> temp{};
	std::reverse_copy(bytes, bytes + sizeof(t), temp.bytes);
	return temp.data;
}

template<class pack_info_t, class data_t>
inline auto pack_into(const msg_union<data_t> &value) -> decltype(pack<pack_info_t>()) {
	std::array<uint8_t, 8> buffer{};
	std::reverse_copy(value.bytes, value.bytes + sizeof(data_t), buffer.data());
	return pack<pack_info_t>(std::array<uint8_t, 8>(buffer));
}

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
		x = d * sin;
		y = d * cos;
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

inline void operator+=(odometry_t &odometry,
                       odometry_update_info<> info) {
	odometry.s += (info.d_left + info.d_rigth) / 2;
	
	double theta, x, y;
	calculate_odometry(info.d_left, info.d_rigth, theta, x, y);
	rotate(x, y, odometry.theta);
	
	odometry.x += x;
	odometry.y += y;
	odometry.theta += theta;
	
	odometry.vx = x / info.d_t.count();
	odometry.vy = y / info.d_t.count();
	odometry.w  = theta / info.d_t.count();
}
