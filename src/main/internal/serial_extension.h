//
// Created by ydrml on 2019/3/12.
//

#ifndef PM1_SDK_SERIAL_EXTENSION_H
#define PM1_SDK_SERIAL_EXTENSION_H


#include <memory>
#include "serial/serial.h"
#include "can/pack.h"

namespace autolabor {
	template<class t>
	inline serial::Serial &operator<<(serial::Serial &port,
	                                  const autolabor::can::msg_union<t> &msg) {
		port.write(msg.bytes, sizeof(t));
		return port;
	}
}


#endif //PM1_SDK_SERIAL_EXTENSION_H
