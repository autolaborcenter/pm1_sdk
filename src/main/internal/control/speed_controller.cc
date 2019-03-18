//
// Created by ydrml on 2019/3/18.
//

#include <cmath>
#include <exception>
#include "speed_controller.hh"

using namespace autolabor::pm1;

speed_controller::speed_controller(
		double k,
		double dead_area,
		double min_speed,
		double max_speed) : k(k),
                            dead_area(std::fmax(0, dead_area)),
                            min_speed(min_speed),
                            max_speed(max_speed) {
	if (min_speed > max_speed)
		throw std::exception("illegal speed range");
}

double speed_controller::operator()(double error) const {
	return error < -dead_area
	       ? std::fmax(k * error, -max_speed)
	       : error < 0
	         ? -min_speed
	         : error == 0
	           ? 0
	           : error < dead_area
	             ? min_speed
	             : std::fmin(k * error, max_speed);
}


