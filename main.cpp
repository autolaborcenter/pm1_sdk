//
// Created by ydrml on 2019/2/22.
//

#include <thread>
#include <iostream>
#include "pm1/extensions.h"
#include "pm1/time_extensions.h"
#include "pm1/api.h"
#include "pm1/internal/chassis.h"

using namespace mechdancer::common;

template<class _, class __>
inline void println(std::chrono::duration<_, __> duration) {
	println(duration.count());
}

int main() {
	println(join_to_string("", 1, 2, 3, 4, 5));
	println(join_to_string(", ", 1, 2, 3, 4, 5));
	println(join_to_string("", '[', join_to_string(", ", 1, 2, 3, 4, 5), ']'));
	
	println(measure_time([] { autolabor::pm1::delay(1); }));
	
	const auto chassis = autolabor::pm1::chassis::instance();
	std::cout << chassis << std::endl;
	
	return 0;
}
