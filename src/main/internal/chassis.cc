//
// Created by ydrml on 2019/2/25.
//

#include "chassis.hh"
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

const auto my_timeout = serial::Timeout(serial::Timeout::max(), 1, 0, 0, 0); // NOLINT(cert-err58-cpp)

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

inline void loop_sleep() {
	std::this_thread::sleep_for(std::chrono::milliseconds(period / 2));
}

inline int first_int(const uint8_t *bytes) {
	return msg_union<int>{bytes[3], bytes[2], bytes[1], bytes[0]}.data;
}

inline short first_short(const uint8_t *bytes) {
	return msg_union<short>{bytes[1], bytes[0]}.data;
}

chassis::chassis(const std::string &port_name)
		: port(new serial::Serial(port_name, 115200, my_timeout)) {
	// 启动接收线程
	std::thread([this] {
		auto        port_ptr = port;
		std::string buffer;
		parser      parser;
		
		// 设置超时时间：200 ms
		write(port_ptr, pack<ecu<>::timeout>({2, 0}));
		
		auto time = mechdancer::common::now();
		
		while (port_ptr->isOpen()) {
			try { buffer = port->read(); }
			catch (std::exception &) { buffer = ""; }
			
			// 每 90 ms 询问一次
			const auto now = mechdancer::common::now();
			if (now - time > std::chrono::milliseconds(period)) {
				time = now;
				try { ask_state(port_ptr); }
				catch (std::exception &) {}
			}
			
			// 处理
			if (buffer.empty() && port_ptr->isOpen()) {
				loop_sleep();
			} else {
				auto result = parser(*buffer.begin());
				if (result.type != parser::result_type::message)
					continue;
				
				const auto msg   = result.message;
				const auto bytes = msg.data.data;
				
				if (ecu0_speed::match(msg))
					_left.speed = first_int(bytes) * mechanical::wheel_k;
				
				else if (ecu0_position::match(msg))
					_left.position = first_int(bytes) * mechanical::wheel_k;
				
				else if (ecu1_speed::match(msg))
					_right.speed = first_int(bytes) * mechanical::wheel_k;
				
				else if (ecu1_position::match(msg))
					_right.position = first_int(bytes) * mechanical::wheel_k;
				
				else if (tcu0_speed::match(msg))
					_rudder.speed = first_short(bytes) * mechanical::rudder_k;
				
				else if (tcu0_position::match(msg)) {
					_rudder.position = first_short(bytes) * mechanical::rudder_k;
					std::cout << "tcu0 = " << _rudder.position << std::endl;
				}
			}
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
	msg_union<int> buffer{};
	buffer.data = static_cast<int> (target / mechanical::wheel_k);
	write(port, pack<ecu0_target>({buffer.bytes[3],
	                               buffer.bytes[2],
	                               buffer.bytes[1],
	                               buffer.bytes[0]}));
}

void chassis::right(double target) const {
	msg_union<int> buffer{};
	buffer.data = static_cast<int> (target / mechanical::wheel_k);
	write(port, pack<ecu1_target>({buffer.bytes[3],
	                               buffer.bytes[2],
	                               buffer.bytes[1],
	                               buffer.bytes[0]}));
}

void chassis::rudder(double target) const {
	msg_union<short> buffer{};
	buffer.data = static_cast<short> (target / mechanical::rudder_k);
	write(port, pack<tcu0_target>({buffer.bytes[1],
	                               buffer.bytes[0]}));
}
