constexpr auto
	chassis_pointer_busy = "chassis pointer is busy",
	null_chassis_pointer = "null chassis pointer",
	infinite_action      = "action never complete",
	invalid_target       = "invalid target",
	action_canceled      = "action canceled";

//
// Created by User on 2019/4/10.
//

#include "pm1_sdk_native.h"

#include <atomic>
#include <algorithm>
#include <sstream>
#include <unordered_set>

#include "exception_engine.hh"
#include "safe_shared_ptr.hh"

#include "internal/serial/serial.h"
#include "internal/chassis.hh"
#include "internal/raii/weak_shared_lock.hh"
#include "internal/process_controller.hh"
#include "internal/raii/weak_lock_guard.hh"

using handler_t = autolabor::pm1::native::handler_t;

std::atomic<handler_t> task_id(0);

autolabor::exception_engine exceptions; // NOLINT(cert-err58-cpp)

safe_shared_ptr<autolabor::pm1::chassis> chassis_ptr;
using ptr_t = decltype(chassis_ptr)::ptr_t;

std::atomic<autolabor::odometry_t>
	odometry_mark         = ATOMIC_VAR_INIT({});

template<class t>
inline handler_t read_ptr(std::function < t(ptr_t) > && block) {
	handler_t id = ++task_id;
	try {
		chassis_ptr.read<t>(std::move(block));
	} catch (std::exception &e) {
		exceptions.set(id, e.what());
	}
	return id;
}

const char *
STD_CALL autolabor::pm1::native::
get_error_info(handler_t handler) {
	return exceptions[handler];
}

void
STD_CALL autolabor::pm1::native::
remove_error_info(handler_t handler) {
	exceptions.remove(handler);
}

void
STD_CALL autolabor::pm1::native::
clear_error_info() {
	exceptions.clear();
}

std::vector<std::string> serial_ports() {
	auto                     info = serial::list_ports();
	std::vector<std::string> result(info.size());
	std::transform(info.begin(), info.end(), result.begin(),
	               [](const serial::PortInfo &it) { return it.port; });
	return result;
}

handler_t
STD_CALL autolabor::pm1::native::
initialize(const char *port, double *const progress) {
	handler_t id = ++task_id;
	if (progress) *progress = 0;
	
	auto list = port == nullptr || std::strlen(port) == 0
	            ? serial_ports()
	            : std::vector{std::string(port)};
	
	if (list.empty())
		exceptions.set(id, "no available port");
	else {
		std::stringstream builder;
		
		for (auto i = list.begin(); i < list.end(); ++i) {
			if (progress) *progress = static_cast<double>(i - list.begin()) / list.size();
			try {
				auto ptr = std::make_shared<chassis>(*i);
				odometry_mark = ptr->odometry();
				chassis_ptr(ptr);
				exceptions.remove(id);
				break;
			}
			catch (std::exception &e) {
				builder << *i << " : " << e.what() << std::endl;
				exceptions.set(id, builder.str());
			}
		}
	}
	
	if (progress) *progress = 1;
	return id;
}

handler_t
STD_CALL autolabor::pm1::native::
shutdown() {
	handler_t id = ++task_id;
	if (!chassis_ptr(nullptr))
		exceptions.set(id, null_chassis_pointer);
	return id;
}

handler_t
STD_CALL autolabor::pm1::native::
get_odometry(double &s, double &sa,
             double &x, double &y, double &theta,
             double &vx, double &vy, double &w) {
	handler_t id = ++task_id;
	try {
		chassis_ptr.read<void>([&](ptr_t ptr) {
			auto temp = ptr->odometry() - odometry_mark;
			s     = temp.s;
			sa    = temp.sa;
			x     = temp.x;
			y     = temp.y;
			theta = temp.theta;
			vx    = temp.vx;
			vy    = temp.vy;
			w     = temp.w;
		});
	} catch (std::exception &e) {
		s     = NAN;
		sa    = NAN;
		x     = NAN;
		y     = NAN;
		theta = NAN;
		vx    = NAN;
		vy    = NAN;
		w     = NAN;
		exceptions.set(id, e.what());
	}
	return id;
}

handler_t
STD_CALL autolabor::pm1::native::
reset_odometry() {
	return read_ptr<void>([](ptr_t ptr) {
		odometry_mark = ptr->odometry();
	});
}

handler_t
STD_CALL autolabor::pm1::native::
lock() {
	return read_ptr<void>([](ptr_t ptr) {
		ptr->disable();
	});
}

handler_t
STD_CALL autolabor::pm1::native::
unlock() {
	return read_ptr<void>([](ptr_t ptr) {
		ptr->enable();
	});
}

handler_t
STD_CALL autolabor::pm1::native::
check_state(unsigned char &what) {
	return read_ptr<void>([&what](ptr_t ptr) {
		auto states = ptr->state().as_vector();
		what = 1 == std::unordered_set(states.begin(), states.end()).size()
		       ? static_cast<unsigned char>(states.front())
		       : 0x7f;
	});
}

handler_t
STD_CALL autolabor::pm1::native::
drive(double v, double w) {
	velocity temp{static_cast<float>(v), static_cast<float>(w)};
	
	return read_ptr<void>(
		[physical = velocity_to_physical(&temp, &default_config)]
			(ptr_t ptr) { ptr->set_target(physical.speed, physical.rudder); });
}

std::mutex    action_mutex;
volatile bool pause_flag  = false,
              cancel_flag = false;

//void
//block(double v,
//      double w,
//      double limit,
//      const autolabor::process_controller &move_controller,
//      std::function<double(ptr_t)> &&current_getter) {
//	weak_lock_guard<decltype(action_mutex)> lk1(action_mutex);
//	if (!lk1) throw std::exception("another action is invoking");
//
//	autolabor::process_t process{};
//	process.begin     = read_ptr<double>([&](ptr_t ptr) { return current_getter(ptr); });
//	process.end       = process.begin + limit;
//
//	velocity   temp{static_cast<float>(v), static_cast<float>(w)};
//	const auto target = velocity_to_physical(&temp, &default_config);
//	process.speed = target.speed;
//
//	auto paused = pause_flag;
//	while (!cancel_flag) {
//		if (pause_flag) {
//			if (!paused) {
//				paused = true;
//				read_ptr<void>([](ptr_t ptr) {
//					ptr->set_target(0, NAN);
//				});
//			}
//		} else {
//			auto finished = read_ptr<bool>([&](ptr_t ptr) {
//				using namespace autolabor::pm1;
//
//				auto states = ptr->state();
//				if (states.check_all(node_state_t::disabled))
//					throw std::exception("chassis is locked");
//
//				auto current = current_getter(ptr);
//				if (current > process.end) return true;
//
//				if (paused) {
//					paused = false;
//					process.begin = current;
//				}
//				ptr->set_target(std::abs(target.rudder - ptr->rudder().position) > 2E-3
//				                ? 0
//				                : move_controller(process, current),
//				                target.rudder);
//				return false;
//			});
//			if (finished) break;
//		}
//
//		std::this_thread::sleep_for(std::chrono::milliseconds(50));
//	}
//
//	read_ptr<void>([](ptr_t ptr) {
//		ptr->set_target(0, NAN);
//	});
//}
