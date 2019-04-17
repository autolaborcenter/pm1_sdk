//
// Created by User on 2019/4/16.
//

#include "pm1_sdk_native.h"

#include <atomic>

#include "internal/exception_engine.hh"
#include "internal/odometry_t.hh"
#include "internal/time_extensions.h"

extern "C" {
#include "internal/control_model/chassis_config_t.h"
}

using handler_t = autolabor::pm1::native::handler_t;

std::atomic<handler_t> task_id(0);

autolabor::exception_engine exceptions; // NOLINT(cert-err58-cpp)

volatile unsigned char state = 0;

std::mutex    action_mutex;
volatile bool pause_flag     = false,
              cancel_flag    = false;

std::atomic<autolabor::odometry_t>
	odometry_inject = ATOMIC_VAR_INIT({});

const char *
STD_CALL autolabor::pm1::native::
get_error_info(handler_t handler) noexcept {
	return exceptions[handler];
}

void
STD_CALL autolabor::pm1::native::
remove_error_info(handler_t handler) noexcept {
	exceptions.remove(handler);
}

void
STD_CALL autolabor::pm1::native::
clear_error_info() noexcept {
	exceptions.clear();
}

std::string current_port;

const char *
STD_CALL autolabor::pm1::native::
get_current_port() noexcept {
	return current_port.c_str();
}

handler_t
STD_CALL autolabor::pm1::native::
initialize(const char *port,
           double width,
           double length,
           double wheel_radius,
           double optimize_width,
           double acceleration,
           double &progress) noexcept {
	handler_t id = ++task_id;
	current_port = port == nullptr || std::strlen(port) == 0
	               ? "COM99"
	               : std::string(port);
	odometry_inject.store({});
	progress = 1;
	state    = 1;
	return id;
}

handler_t
STD_CALL autolabor::pm1::native::
shutdown() noexcept {
	handler_t id = ++task_id;
	if (current_port.empty())
		exceptions.set(id, "null chassis pointer");
	current_port.clear();
	state = 0;
	return id;
}

handler_t
STD_CALL autolabor::pm1::native::
get_odometry(double &s, double &sa,
             double &x, double &y, double &theta,
             double &vx, double &vy, double &w) noexcept {
	handler_t id       = ++task_id;
	auto      odometry = odometry_inject.load();
	s     = odometry.s;
	sa    = odometry.sa;
	x     = odometry.x;
	y     = odometry.y;
	theta = odometry.theta;
	vx    = odometry.vx;
	vy    = odometry.vy;
	w     = odometry.w;
	return id;
}

handler_t
STD_CALL autolabor::pm1::native::
reset_odometry() noexcept {
	odometry_inject.store({});
	return ++task_id;
}

handler_t
STD_CALL autolabor::pm1::native::
lock() noexcept {
	state = 0xff;
	return ++task_id;
}

handler_t
STD_CALL autolabor::pm1::native::
unlock() noexcept {
	state = 1;
	return ++task_id;
}

unsigned char
STD_CALL autolabor::pm1::native::
check_state() noexcept {
	return state;
}

handler_t
STD_CALL autolabor::pm1::native::
drive_physical(double, double) noexcept {
	return ++task_id;
}

handler_t
STD_CALL autolabor::pm1::native::
drive(double, double) noexcept {
	return ++task_id;
}

double
STD_CALL autolabor::pm1::native::
spatium_calculate(double spatium, double angle) noexcept {
	const static auto w_2 = default_config.width / 2;
	
	return std::abs(spatium + w_2 * angle) +
	       std::abs(spatium - w_2 * angle);
}

handler_t
STD_CALL autolabor::pm1::native::
drive_spatial(double v,
              double w,
              double spatium,
              double &progress) noexcept {
	handler_t id = ++task_id;
	progress = 1;
	return id;
}

handler_t
STD_CALL autolabor::pm1::native::
drive_timing(double v,
             double w,
             double time,
             double &progress) noexcept {
	handler_t id = ++task_id;
	progress = 1;
	return id;
}

handler_t
STD_CALL autolabor::pm1::native::
adjust_rudder(double offset,
              double &progress) noexcept {
	handler_t id = ++task_id;
	progress = 1;
	return id;
}

void
STD_CALL autolabor::pm1::native::
pause() noexcept { pause_flag = true; }

void
STD_CALL autolabor::pm1::native::
resume() noexcept { pause_flag = false; }

bool
STD_CALL autolabor::pm1::native::
is_paused() noexcept { return pause_flag; }

void
STD_CALL autolabor::pm1::native::
cancel_all() noexcept {
	cancel_flag = true;
	{ std::lock_guard<std::mutex> wait(action_mutex); }
	cancel_flag = false;
}
