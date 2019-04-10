//
// Created by User on 2019/4/9.
//

#include "exception_engine.hh"

void autolabor::exception_engine::set(size_t id, const std::string &text) {
	std::unique_lock<decltype(mutex)> _(mutex);
	map[id] = text;
}

void autolabor::exception_engine::remove(size_t id) {
	std::unique_lock<decltype(mutex)> _(mutex);
	map.erase(id);
}

void autolabor::exception_engine::clear() {
	std::unique_lock<decltype(mutex)> _(mutex);
	map.clear();
}

const char *autolabor::exception_engine::operator[](size_t id) const {
	std::shared_lock<decltype(mutex)> _(mutex);
	
	auto ptr = map.find(id);
	return ptr == map.end() ? nothing
	                        : ptr->second.c_str();
}
