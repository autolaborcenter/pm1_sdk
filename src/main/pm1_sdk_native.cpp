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

using handler_t = autolabor::pm1::native::handler_t;

std::atomic<handler_t> task_id(0);

autolabor::exception_engine exceptions; // NOLINT(cert-err58-cpp)

safe_shared_ptr<autolabor::pm1::chassis> chassis_ptr;
using ptr_t = decltype(chassis_ptr)::ptr_t;

std::atomic<autolabor::odometry_t>
	odometry_mark         = ATOMIC_VAR_INIT({});

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

std::vector<std::string> serial_ports() {
	auto                     info = serial::list_ports();
	std::vector<std::string> result(info.size());
	std::transform(info.begin(), info.end(), result.begin(),
	               [](const serial::PortInfo &it) { return it.port; });
	return result;
}

std::string current_port;

const char *
STD_CALL autolabor::pm1::native::
get_current_port() noexcept {
	return current_port.c_str();
}

handler_t
STD_CALL autolabor::pm1::native::
initialize(const char *port, double &progress) noexcept {
	handler_t id = ++task_id;
	progress = 0;
	
	auto list = port == nullptr || std::strlen(port) == 0
	            ? serial_ports()
	            : std::vector<std::string>{std::string(port)};
	
	if (list.empty())
		exceptions.set(id, "no available port");
	else {
		std::stringstream builder;
		
		for (auto i = list.begin(); i < list.end(); ++i) {
			progress = static_cast<double>(i - list.begin()) / list.size();
			try {
				auto ptr = std::make_shared<chassis>(*i);
				odometry_mark = ptr->odometry();
				current_port = *i;
				chassis_ptr(ptr);
				break;
			}
			catch (std::exception &e) {
				builder << *i << " : " << e.what() << std::endl;
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

handler_t
STD_CALL autolabor::pm1::native::
check_state(unsigned char &what) noexcept {
	return use_ptr([&what](ptr_t ptr) {
		auto states = ptr->state().as_vector();
		what = 1 == std::unordered_set<node_state_t>(states.begin(), states.end()).size()
		       ? static_cast<unsigned char>(states.front())
		       : 0x7f;
	});
}

handler_t
STD_CALL autolabor::pm1::native::
drive(double v, double w) noexcept {
	velocity temp{static_cast<float>(v), static_cast<float>(w)};
	
	return use_ptr(
		[physical = velocity_to_physical(&temp, &default_config)]
			(ptr_t ptr) { ptr->set_target(physical.speed, physical.rudder); });
}

std::mutex    action_mutex;
volatile bool pause_flag  = false,
              cancel_flag = false;

handler_t block(double v,
                double w,
                double limit,
                const autolabor::process_controller &controller,
                std::function<double(ptr_t)> &&measure,
                double &progress) noexcept {
	handler_t id    = ++task_id;
	
	velocity temp{static_cast<float>(v), static_cast<float>(w)};
	auto     target = velocity_to_physical(&temp, &default_config);
	
	weak_lock_guard<decltype(action_mutex)> lock(action_mutex);
	if (!lock) {
		exceptions.set(id, "another action is invoking");
		return id;
	}
	
	autolabor::process_t process{0, 0, target.speed};
	progress = 0;
	
	auto rest     = 1 - progress;
	auto paused   = true;
	auto finished = false;
	try {
		while (!finished) {
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
				finished = chassis_ptr.read<bool>([&](ptr_t ptr) {
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
						ptr->set_target(std::abs(target.rudder - ptr->rudder().position) > 2E-3
						                ? 0
						                : controller(process, current),
						                target.rudder);
					return false;
				});
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
		}
		
		chassis_ptr.read<void>([](ptr_t ptr) { ptr->set_target(0, NAN); });
	} catch (const std::exception &e) {
		exceptions.set(id, e.what());
	}
	
	return id;
}

handler_t
STD_CALL autolabor::pm1::native::
drive_spatial(double v,
              double w,
              double spatium,
              double &progress) noexcept {
	return block(v, w, spatium,
	             {0.5, 0.1, 12, 4},
	             [](ptr_t ptr) {
		             const static auto w_2 = default_config.width / 2;
		
		             auto odometry = ptr->odometry();
		             return std::abs(odometry.s + w_2 * odometry.sa) +
		                    std::abs(odometry.s - w_2 * odometry.sa);
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

void
STD_CALL autolabor::pm1::native::
pause() noexcept { pause_flag = true; }

void
STD_CALL autolabor::pm1::native::
resume() noexcept { pause_flag = false; }

void
STD_CALL autolabor::pm1::native::
cancel_all() noexcept {
	cancel_flag = true;
	{ std::lock_guard<std::mutex> wait(action_mutex); }
	cancel_flag = false;
}
