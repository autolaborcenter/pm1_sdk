//
// Created by ydrml on 2019/3/12.
//

#ifndef PM1_SDK_SERIAL_EXTENSION_H
#define PM1_SDK_SERIAL_EXTENSION_H


#include <vector>
#include "can/pack.h"
#include "serial/serial_port.hh"

namespace autolabor {
	template<class t>
	inline serial_port &operator<<(serial_port &port,
	                               const autolabor::can::msg_union<t> &msg) {
		port.send(msg.bytes, sizeof(t));
		return port;
	}
}


#endif //PM1_SDK_SERIAL_EXTENSION_H
