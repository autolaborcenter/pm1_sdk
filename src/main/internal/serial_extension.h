//
// Created by ydrml on 2019/3/12.
//

#ifndef PM1_SDK_SERIAL_EXTENSION_H
#define PM1_SDK_SERIAL_EXTENSION_H


#include <string>
#include "serial/serial.h"
#include "can/pack.h"

namespace autolabor {
	template<class t>
	inline serial::Serial &operator<<(serial::Serial &port,
	                                  const autolabor::can::msg_union<t> &msg) {
		try { port.write(msg.bytes, sizeof(t)); } catch (...) {}
		return port;
	}
	
	inline serial::Serial &operator>>(serial::Serial &port,
	                                  std::string &msg) {
		try { msg = port.read(); } catch (...) { msg = ""; }
		return port;
	}
}


#endif //PM1_SDK_SERIAL_EXTENSION_H
