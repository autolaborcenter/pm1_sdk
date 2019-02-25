//
// Created by ydrml on 2019/2/22.
//

#include "api.h"

#include <string>
#include <stdexcept>
#include <iostream>
#include <thread>
#include "extensions.h"
#include "time_extensions.h"

namespace {
	using namespace autolabor::pm1;
	using namespace mechdancer::common;
	using seconds = std::chrono::duration<double, std::ratio<1>>;
	
	// 记录暂停状态
	volatile bool paused = false;
	
	// 循环的间隔
	inline void loop_delay() { delay(0.05); }
	
	// 阻塞并抑制输出
	seconds inhibit() {
		const auto temp = now();
		while (paused) {
			drive(0, 0);
			loop_delay();
		}
		return now() - temp;
	}
	
	// 按固定控制量运行指定时间
	void go_timing(double v, double w, double seconds) {
		auto ending = now() + seconds_duration(seconds);
		while (now() < ending) {
			if (paused) ending += inhibit();
			else drive(v, w);
			
			loop_delay();
		}
	}
}

bool autolabor::pm1::initialize(std::string port) {
	throw std::runtime_error("to do");
}

bool autolabor::pm1::initialize() {
	throw std::runtime_error("to do");
}

void autolabor::pm1::shutdown() {
	throw std::runtime_error("to do");
}

void autolabor::pm1::go_straight(double speed, double distance) {
	while (false) {
		if (paused) autolabor::pm1::drive(0, 0);
		else autolabor::pm1::drive(speed, speed);
		
		loop_delay();
	}
}

void autolabor::pm1::go_straight_timing(double speed, double time) {
	go_timing(speed, speed, time);
}

void autolabor::pm1::go_arc(double speed, double r, double rad) {
	while (false) {
		if (paused) drive(0, 0);
		else drive(speed, speed / r);
		
		loop_delay();
	}
}

void autolabor::pm1::go_arc_timing(double speed, double r, double time) {
	go_timing(speed, speed / r, time);
}

void autolabor::pm1::turn_around(double speed, double rad) {
	while (false) {
		if (paused) drive(0, 0);
		else drive(0, speed);
		
		loop_delay();
	}
}

void autolabor::pm1::turn_around_timing(double speed, double time) {
	go_timing(0, speed, time);
}

void autolabor::pm1::pause() {
	drive(0, 0);
	paused = true;
}

void autolabor::pm1::resume() {
	paused = false;
}

void autolabor::pm1::delay(double time) {
	std::this_thread::sleep_for(mechdancer::common::seconds_duration(time));
}

autolabor::pm1::Odometry autolabor::pm1::get_odometry() {
	throw std::runtime_error("to do");
}

void autolabor::pm1::drive(double v, double w) {
	std::cout << "v = " << v << ", w = " << w << std::endl;
}
