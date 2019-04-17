//
// Created by User on 2019/4/10.
//

#include "pm1_sdk_native.h"

#include <atomic>
#include <vector>
#include <algorithm>
#include <sstream>
#include <unordered_set>

#include "internal/exception_engine.hh"
#include "internal/raii/safe_shared_ptr.hh"

#include "internal/chassis.hh"
#include "internal/serial/serial.h"
#include "internal/raii/weak_shared_lock.hh"
#include "internal/process_controller.hh"
#include "internal/raii/weak_lock_guard.hh"

// region task resources

using handler_t = autolabor::pm1::native::handler_t;

std::atomic<handler_t> task_id(0);

autolabor::exception_engine exceptions; // NOLINT(cert-err58-cpp)

// endregion
// region chassis resources

safe_shared_ptr<autolabor::pm1::chassis> chassis_ptr;
using ptr_t = decltype(chassis_ptr)::ptr_t;

std::atomic<autolabor::odometry_t>
	odometry_mark         = ATOMIC_VAR_INIT({});

// endregion
// region action resource

std::mutex    action_mutex;
volatile bool pause_flag  = false,
              cancel_flag = false;

// endregion

inline handler_t use_ptr(std::function < void(ptr_t) > && block) {
	handler_t id = ++task_id;
	try {
		chassis_ptr.read<void>(block);
	} catch (std::exception &e) {
		exceptions.set(id, e.what());
	}
	return id;
}

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

void
STD_CALL autolabor::pm1::native::
get_default_chassis_config(double &width,
                           double &length,
                           double &wheel_radius,
                           double &optimize_width,
                           double &acceleration) noexcept {
	width          = default_config.width;
	length         = default_config.length;
	wheel_radius   = default_config.radius;
	optimize_width = pi_f / 4;
	acceleration   = 2 * pi_f;
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
	const static auto serial_ports = [] {
		auto                     info = serial::list_ports();
		std::vector<std::string> result(info.size());
		std::transform(info.begin(), info.end(), result.begin(),
		               [](const serial::PortInfo &it) { return it.port; });
		return result;
	};
	
	handler_t id = ++task_id;
	progress = 0;
	
	const static auto if_nan = [](double expect, double default_value) {
		return static_cast<float>(std::isnan(expect) ? default_value : expect);
	};
	
	chassis_config_t config{if_nan(width, default_config.width),
	                        if_nan(length, default_config.length),
	                        if_nan(wheel_radius, default_config.radius)};
	
	auto list = port == nullptr || std::strlen(port) == 0
	            ? serial_ports()
	            : std::vector<std::string>{std::string(port)};
	
	if (list.empty())
		exceptions.set(id, "no available port");
	else {
		std::stringstream builder;
		
		for (auto i = list.begin();;) {
			progress = static_cast<double>(i - list.begin()) / list.size();
			try {
				auto ptr = std::make_shared<chassis>
					(*i, config,
					 if_nan(optimize_width, pi_f / 4),
					 if_nan(acceleration, 2 * pi_f));
				
				chassis_ptr(ptr);
				builder.str("");
				odometry_mark = ptr->odometry();
				current_port  = *i;
				pause_flag    = false;
				cancel_flag   = false;
				break;
			}
			catch (std::exception &e) {
				builder << *i << " : " << e.what();
				if (++i < list.end()) builder << std::endl;
				else break;
			}
		}
		exceptions.set(id, builder.str());
	}
	
	progress = 1;
	return id;
}

handler_t
STD_CALL autolabor::pm1::native::
shutdown() noexcept {
	handler_t id = ++task_id;
	if (!chassis_ptr(nullptr))
		exceptions.set(id, "null chassis pointer");
	current_port.clear();
	return id;
}

handler_t
STD_CALL autolabor::pm1::native::
get_odometry(double &s, double &sa,
             double &x, double &y, double &theta,
             double &vx, double &vy, double &w) noexcept {
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
reset_odometry() noexcept {
	return use_ptr([](ptr_t ptr) {
		odometry_mark = ptr->odometry();
	});
}

handler_t
STD_CALL autolabor::pm1::native::
lock() noexcept {
	return use_ptr([](ptr_t ptr) {
		ptr->disable();
	});
}

handler_t
STD_CALL autolabor::pm1::native::
unlock() noexcept {
	return use_ptr([](ptr_t ptr) {
		ptr->enable();
	});
}

unsigned char
STD_CALL autolabor::pm1::native::
check_state() noexcept {
	try {
		return chassis_ptr.read<unsigned char>([](ptr_t ptr) {
			auto states = ptr->state().as_vector();
			return 1 == std::unordered_set<node_state_t>(states.begin(), states.end()).size()
			       ? static_cast<unsigned char>(states.front())
			       : 0x7f;
		});
	} catch (std::exception &) {
		return 0;
	}
}

handler_t
STD_CALL autolabor::pm1::native::
drive_physical(double speed, double rudder) noexcept {
	handler_t id = ++task_id;
	
	weak_lock_guard<decltype(action_mutex)> lock(action_mutex);
	if (!lock) {
		exceptions.set(id, "another action is invoking");
		return id;
	}
	
	try {
		chassis_ptr.read<void>([=](ptr_t ptr) { ptr->set_target(speed, rudder); });
	} catch (std::exception &e) {
		exceptions.set(id, e.what());
	}
	return id;
}

handler_t
STD_CALL autolabor::pm1::native::
drive(double v, double w) noexcept {
	velocity temp{static_cast<float>(v), static_cast<float>(w)};
	auto     physical = velocity_to_physical(&temp, &default_config);
	return drive_physical(physical.speed, physical.rudder);
}

handler_t block(double v,
                double w,
                double limit,
                const autolabor::process_controller &controller,
                std::function<double(ptr_t)> &&measure,
                double &progress) noexcept {
	handler_t id    = ++task_id;
	
	velocity temp{static_cast<float>(v), static_cast<float>(w)};
	auto     target = velocity_to_physical(&temp, &default_config);
	
	autolabor::process_t process{0, 0, target.speed};
	progress = 0;
	
	weak_lock_guard<decltype(action_mutex)> lock(action_mutex);
	if (!lock) {
		exceptions.set(id, "another action is invoking");
		return id;
	}
	
	auto rest   = 1 - progress;
	auto paused = true;
	try {
		while (true) {
			if (cancel_flag) {
				chassis_ptr.read<void>([](ptr_t ptr) { ptr->set_target(0, NAN); });
				throw std::exception("action canceled");
			}
			
			if (paused) {
				// 检查恢复标记
				if (!(paused = pause_flag)) {
					process.begin = chassis_ptr.read(measure);
					process.end   = process.begin + limit;
				}
			} else {
				auto finished = chassis_ptr.read<bool>([&](ptr_t ptr) {
					constexpr static auto disabled = autolabor::pm1::node_state_t::disabled;
					
					// 检查状态
					auto states = ptr->state();
					if (!states.check_all())
						throw std::exception(states.check_all(disabled)
						                     ? "chassis is locked"
						                     : "critical error");
					// 检查任务进度
					auto current = measure(ptr);
					auto sub     = process[current];
					// 任务完成
					if ((progress = 1 - rest * (1 - sub)) >= 1)
						return true;
					// 检查暂停标记
					if ((paused   = pause_flag)) {
						limit *= (1 - sub); // 子任务规模缩减
						rest *= (1 - sub);  // 子任务比例缩减
						ptr->set_target(0, target.rudder);
					} else
						ptr->set_target(std::abs(target.rudder - ptr->rudder().position) < pi_f / 120
						                ? controller(process, current)
						                : 0,
						                target.rudder);
					return false;
				});
				
				if (finished) break;
			}
			using namespace std::chrono_literals;
			std::this_thread::sleep_for(50ms);
		}
		
		chassis_ptr.read<void>([](ptr_t ptr) { ptr->set_target(0, NAN); });
	} catch (const std::exception &e) {
		exceptions.set(id, e.what());
	}
	
	return id;
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
	return block(v, w, spatium,
	             {0.5, 0.1, 12, 4},
	             [origin = chassis_ptr.read<odometry_t>([](ptr_t ptr) { return ptr->odometry(); })]
		             (ptr_t ptr) {
		             auto odometry = ptr->odometry() - origin;
		             return spatium_calculate(odometry.s, odometry.sa);
	             },
	             progress);
}

handler_t
STD_CALL autolabor::pm1::native::
drive_timing(double v,
             double w,
             double time,
             double &progress) noexcept {
	return block(v, w, time,
	             {0.5, 0.1, 5, 2},
	             [](ptr_t) {
		             return std::chrono::duration_cast<seconds_floating>(
			             now().time_since_epoch()
		             ).count();
	             },
	             progress);
}

handler_t
STD_CALL autolabor::pm1::native::
adjust_rudder(double offset,
              double &progress) noexcept {
	handler_t id = ++task_id;
	
	weak_lock_guard<decltype(action_mutex)> lock(action_mutex);
	if (!lock) {
		exceptions.set(id, "another action is invoking");
		return id;
	}
	
	const auto total = chassis_ptr.read<double>([=](ptr_t ptr) {
		ptr->set_target(0, offset);
		return std::abs(offset - ptr->rudder().position);
	});
	
	try {
		while (true) {
			using namespace std::chrono_literals;
			
			if (cancel_flag) {
				chassis_ptr.read<void>([](ptr_t ptr) { ptr->set_target(0, NAN); });
				throw std::exception("action canceled");
			}
			
			if (pause_flag)
				chassis_ptr.read<void>([](ptr_t ptr) { ptr->set_target(0, NAN); });
			else {
				auto finished = chassis_ptr.read<bool>([&](ptr_t ptr) {
					auto difference = std::abs(offset - ptr->rudder().position);
					
					if (difference < pi_f / 120) {
						progress = 1;
						std::this_thread::sleep_for(50ms);
						ptr->reset_rudder();
						return true;
					} else {
						progress = difference / total;
						ptr->set_target(0, offset);
						return false;
					}
				});
				
				if (finished) break;
			}
			std::this_thread::sleep_for(50ms);
		}
	} catch (std::exception &e) {
		exceptions.set(id, e.what());
	}
	
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
