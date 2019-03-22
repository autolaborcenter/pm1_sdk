//
// Created by ydrml on 2019/3/22.
//

#ifndef PM1_SDK_LOGGER_HH
#define PM1_SDK_LOGGER_HH

#include <ostream>
#include <chrono>

class time_item {
public:
	const std::chrono::high_resolution_clock::time_point time;
	
	static time_item now();
};

std::ostream &operator<<(std::ostream &ostream, const time_item &time);

#endif //PM1_SDK_LOGGER_HH
