//
// Created by ydrml on 2019/2/25.
//

#include "chassis.hh"
#include "../time_extensions.h"
#include "can_define.h"
#include "parser.hh"

using namespace autolabor::pm1;

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

chassis::chassis(const std::string &port_name)
		: port(new serial::Serial(port_name, 9600, my_timeout)) {
	// 启动接收线程
	std::thread([this] {
		auto        port_ptr = port;
		std::string temp;
		parser      parser;
		
		// 设置超时时间：200 ms
		write(port_ptr, pack<ecu<>::timeout>({2, 0}));
		
		while (port_ptr->isOpen()) {
			try { temp = port->read(1); }
			catch (std::exception &) { temp = ""; }
			
			if (temp.empty() && port_ptr->isOpen())
				mechdancer::common::sleep(50);
			else {
				auto result = parser(*temp.begin());
				switch (result.type) {
					case parser::result_type::nothing:
						break;
					case parser::result_type::signal:
						break;
					case parser::result_type::message:
						break;
				}
			}
		}
	}).detach();
}

chassis::~chassis() {
	port->close();
}
