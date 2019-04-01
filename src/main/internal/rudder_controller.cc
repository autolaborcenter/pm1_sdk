//
// Created by ydrml on 2019/3/12.
//

#include "rudder_controller.hh"

#include "serial_extension.h"
#include "can_define.h"

using namespace autolabor::pm1;

rudder_controller::rudder_controller(const std::string &port_name)
		: port(port_name, 115200), target(0) {}

void rudder_controller::adjust(short additional) {
	target += additional;
	port << pack_big_endian<tcu<0>::target_position>(target);
}

void rudder_controller::done() {
	target = 0;
	port << autolabor::can::pack<tcu<0>::encoder_reset>();
}
