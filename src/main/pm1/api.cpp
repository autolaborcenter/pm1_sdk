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
#include "internal/chassis.hh"

using namespace autolabor::pm1;
using namespace mechdancer::common;
using seconds = std::chrono::duration<double, std::ratio<1>>;

// 记录暂停状态
volatile bool paused = false;

// 循环的间隔
inline void loop_delay() { delay(0.05); }

std::shared_ptr<chassis> chassis_ptr;

/** 阻塞并抑制输出 */
seconds inhibit() {
	const auto temp = now();
	while (paused) {
		drive(0, 0);
		loop_delay();
	}
	return now() - temp;
}

/** 获取编码器均值 */
inline double average_encoders() {
	return (chassis_ptr->left().position + chassis_ptr->right().position) / 2;
}

/** 获取编码器差 */
inline double difference_encoders() {
	return std::abs(chassis_ptr->left().position - chassis_ptr->right().position);
}

/** 阻塞等待后轮转动 */
inline void wait_or(double target_rudder_position,
                    const std::function<void()> &block) {
	if (paused) {
		chassis_ptr->left(0);
		chassis_ptr->right(0);
	} else if (std::abs(chassis_ptr->rudder().position - target_rudder_position) > 0.01) {
		chassis_ptr->left(0);
		chassis_ptr->right(0);
		chassis_ptr->rudder(target_rudder_position);
	} else block();
}

/** 按固定控制量运行指定时间 */
void go_timing(double v, double w, double seconds) {
	auto ending = now() + seconds_duration(seconds);
	while (now() < ending) {
		if (paused) ending += inhibit();
		else drive(v, w);
		
		loop_delay();
	}
}

bool autolabor::pm1::initialize(const std::string &port) {
	try {
		chassis_ptr = std::make_shared<chassis>(port);
		std::cout << "chassis initialized on " << port << std::endl;
	} catch (const std::exception &e) {
		chassis_ptr = nullptr;
		std::cerr << "initialize failed, because: " << e.what() << std::endl;
	}
	return chassis_ptr != nullptr;
}

bool autolabor::pm1::initialize() {
	throw std::runtime_error("to do");
}

void autolabor::pm1::shutdown() {
	chassis_ptr = nullptr;
}

void autolabor::pm1::go_straight(double speed, double distance) {
	const auto o = average_encoders();
	while (std::abs(average_encoders() - o) < distance) {
		if (paused) autolabor::pm1::drive(0, 0);
		else drive(0, 0);
		loop_delay();
	}
}

void autolabor::pm1::go_straight_timing(double speed, double time) {
	go_timing(speed, speed, time);
}

void autolabor::pm1::go_arc(double speed, double r, double rad) {
	const auto o = average_encoders();
	while (std::abs(average_encoders() - o) < r * rad) {
		if (paused) drive(0, 0);
		else drive(speed, speed / r);
		
		loop_delay();
	}
}

void autolabor::pm1::go_arc_timing(double speed, double r, double time) {
	go_timing(speed, speed / r, time);
}

#define width 0

void autolabor::pm1::turn_around(double speed, double rad) {
	const auto o = difference_encoders();
	while (std::abs(difference_encoders() - o) < width * rad) {
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
	// wait_or()
}
