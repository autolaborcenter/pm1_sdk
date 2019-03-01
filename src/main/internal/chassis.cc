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

/** 询问底盘状态 */
inline void ask_state(serial_ref port) {
	write(port, pack<ecu<>::current_speed_tx>());
	write(port, pack<ecu<>::current_position_tx>());
	write(port, pack<tcu<>::current_speed_tx>());
	write(port, pack<tcu<>::current_position_tx>());
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

inline serial::Timeout my_timeout() {
	return serial::Timeout(serial::Timeout::max(), 5, 0, 0, 0);
}

chassis::chassis(const std::string &port_name)
		: port(new serial::Serial(port_name, 115200, my_timeout())) {
	// 启动接收线程
	std::thread([this] {
		auto        port_ptr = port;
		std::string buffer;
		parser      parser;
		
		// 设置超时时间：200 ms
		write(port_ptr, pack<ecu<>::timeout>({2, 0}));
		// 询问初始状态
		ask_state(port_ptr);
		
		auto time    = mechdancer::common::now();
		auto tik_tok = true;
		
		while (port_ptr->isOpen()) {
			// 发送
			const auto now = mechdancer::common::now();
			if (now - time > std::chrono::milliseconds(period / 2)) {
				time         = now;
				if ((tik_tok = !tik_tok)) {
					// 半个周期，询问状态
					try { ask_state(port_ptr); }
					catch (std::exception &) {}
				} else {
					// 另外半个周期，发送指令，计算里程计
					write(port, pack<ecu0_target>({target_left.bytes[3],
					                               target_left.bytes[2],
					                               target_left.bytes[1],
					                               target_left.bytes[0]}));
					write(port, pack<ecu1_target>({target_right.bytes[3],
					                               target_right.bytes[2],
					                               target_right.bytes[1],
					                               target_right.bytes[0]}));
					write(port, pack<tcu0_target>({target_rudder.bytes[1],
					                               target_rudder.bytes[0]}));
				}
			}
			
			// 接收
			try { buffer = port->read(); }
			catch (std::exception &) { buffer = ""; }
			if (buffer.empty()) continue;
			
			// 解析
			auto result = parser(*buffer.begin());
			if (result.type != parser::result_type::message)
				continue;
			
			// 处理
			const auto msg   = result.message;
			const auto bytes = msg.data.data;
			
			if (ecu0_speed::match(msg))
				_left.speed = get_first<int>(bytes) * mechanical::wheel_k;
			
			else if (ecu0_position::match(msg))
				_left.position = get_first<int>(bytes) * mechanical::wheel_k;
			
			else if (ecu1_speed::match(msg))
				_right.speed = get_first<int>(bytes) * mechanical::wheel_k;
			
			else if (ecu1_position::match(msg))
				_right.position = get_first<int>(bytes) * mechanical::wheel_k;
			
			else if (tcu0_speed::match(msg))
				_rudder.speed = get_first<short>(bytes) * mechanical::rudder_k;
			
			else if (tcu0_position::match(msg))
				_rudder.position = get_first<short>(bytes) * mechanical::rudder_k;
		}
	}).detach();
}

chassis::~chassis() {
	port->close();
}

motor_info chassis::left() const {
	return _left;
}

motor_info chassis::right() const {
	return _right;
}

motor_info chassis::rudder() const {
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
