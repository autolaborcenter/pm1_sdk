//
// Created by ydrml on 2019/3/22.
//

#include "logger.hh"

logger::logger(const std::string &name) : file(name) {}

logger::~logger() { file.close(); }

time_item time_item::now() {
	return {std::chrono::high_resolution_clock::now()};
}

std::ostream &operator<<(std::ostream &ostream, const time_item &time) {
	ostream << std::chrono::duration_cast<std::chrono::milliseconds>(time.time.time_since_epoch()).count();
	return ostream;
}
