//
// Created by ydrml on 2019/2/25.
//

#include "chassis.hh"
#include "../time_extensions.h"

using namespace autolabor::pm1;

const auto my_timeout = serial::Timeout(serial::Timeout::max(), 1, 0, 0, 0); // NOLINT(cert-err58-cpp)

chassis::chassis(const std::string &port_name)
		: port(new serial::Serial(port_name, 9600, my_timeout)) {
	// 启动接收线程
	std::thread([this] {
		auto        port_ptr = port;
		std::string temp;
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
