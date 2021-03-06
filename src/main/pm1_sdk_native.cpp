﻿//
// Created by User on 2019/4/10.
//

#include "pm1_sdk_native.h"

#include <atomic>
#include <vector>
#include <algorithm>
#include <sstream>
#include <unordered_set>
#include <cmath>

#include "pm1_sdk_definitions.h"

#include <utilities/raii/safe_shared_ptr.hpp>
#include <utilities/raii/weak_lock_guard.hpp>
#include <utilities/raii/weak_shared_lock.hpp>
#include <utilities/raii/exception_engine.hpp>

#include <utilities/serial_port/serial.h>

#include "internal/chassis.hh"
#include "internal/process_controller.hpp"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "performance-unnecessary-value-param"

constexpr auto action_conflict = "another action is invoking";

// region task resources

using handler_t = autolabor::pm1::native::handler_t;

std::atomic<handler_t> task_id(0);

autolabor::exception_engine<handler_t> exceptions; // NOLINT(cert-err58-cpp)

// endregion
// region chassis resources

safe_shared_ptr<autolabor::pm1::chassis> chassis_ptr;
using ptr_t = decltype(chassis_ptr)::ptr_t;

std::atomic<autolabor::odometry_t<>>
    odometry_mark{};

// endregion
// region action resource

std::mutex    action_mutex;
volatile bool pause_flag       = false,
              cancel_flag      = false;

// endregion

inline handler_t use_ptr(std::function < void(ptr_t) > && block) {
    handler_t id = ++task_id;
    try {
        chassis_ptr.read<void>(block);
    }
    catch (std::exception &e) {
        exceptions.set(id, e.what());
    }
    return id;
}

const char *
STD_CALL
autolabor::pm1::native::
get_error_info(handler_t handler) noexcept {
    return exceptions[handler];
}

void
STD_CALL
autolabor::pm1::native::
remove_error_info(handler_t handler) noexcept {
    exceptions.remove(handler);
}

void
STD_CALL
autolabor::pm1::native::
clear_error_info() noexcept {
    exceptions.clear();
}

std::string connected_port;

const char *
STD_CALL
autolabor::pm1::native::
get_connected_port() noexcept {
    return connected_port.c_str();
}

double
STD_CALL
autolabor::pm1::native::
get_default_parameter(handler_t id) noexcept {
    switch (static_cast<parameter_id>(id)) {
        case parameter_id::length:
            return default_config.length;
        
        case parameter_id::width:
            return default_config.width;
        
        case parameter_id::left_radius:
            return default_config.left_radius;
        
        case parameter_id::right_radius:
            return default_config.right_radius;
        
        case parameter_id::max_wheel_speed:
            return chassis::default_max_wheel_speed;
        
        case parameter_id::max_v:
            return chassis::default_max_v;
        
        case parameter_id::max_w:
            return chassis::default_max_w;
        
        case parameter_id::optimize_width:
            return chassis::default_optimize_width;
        
        case parameter_id::acceleration:
            return chassis::default_acceleration;
        
        default:
            return NAN;
    }
}

handler_t
STD_CALL
autolabor::pm1::native::
get_parameter_c(handler_t id, double *value) noexcept {
    return get_parameter(id, *value);
}

handler_t
STD_CALL
autolabor::pm1::native::
get_parameter(handler_t id, double &value) noexcept {
    return use_ptr([id, &value](ptr_t ptr) {
        switch (static_cast<parameter_id>(id)) {
            case parameter_id::length:
                value = ptr->config.length;
                break;
            case parameter_id::width:
                value = ptr->config.width;
                break;
            case parameter_id::left_radius:
                value = ptr->config.left_radius;
                break;
            case parameter_id::right_radius:
                value = ptr->config.right_radius;
                break;
            case parameter_id::max_wheel_speed:
                value = ptr->max_wheel_speed;
                break;
            case parameter_id::max_v:
                value = ptr->max_v;
                break;
            case parameter_id::max_w:
                value = ptr->max_w;
                break;
            case parameter_id::optimize_width:
                value = ptr->optimize_width;
                break;
            case parameter_id::acceleration:
                value = ptr->acceleration;
                break;
            default:
                throw std::logic_error("undefined id");
        }
    });
}

handler_t
STD_CALL
autolabor::pm1::native::
set_parameter(handler_t id, double value) noexcept {
    return use_ptr([id, temp = static_cast<float>(value)](ptr_t ptr) {
        switch (static_cast<parameter_id>(id)) {
            case parameter_id::length:
                ptr->config.length = temp;
                break;
            case parameter_id::width:
                ptr->config.width = temp;
                break;
            case parameter_id::left_radius:
                ptr->config.left_radius = temp;
                break;
            case parameter_id::right_radius:
                ptr->config.right_radius = temp;
                break;
            case parameter_id::max_wheel_speed:
                ptr->max_wheel_speed = temp;
                break;
            case parameter_id::max_v:
                ptr->max_v = temp;
                break;
            case parameter_id::max_w:
                ptr->max_w = temp;
                break;
            case parameter_id::optimize_width:
                ptr->optimize_width = temp;
                break;
            case parameter_id::acceleration:
                ptr->acceleration = temp;
                break;
            default:
                throw std::logic_error("undefined id");
        }
    });
}

handler_t
STD_CALL
autolabor::pm1::native::
reset_parameter(handler_t id) noexcept {
    return set_parameter(id, get_default_parameter(id));
}

handler_t
STD_CALL
autolabor::pm1::native::
get_battery_percent(double &battery_percent) noexcept {
    return use_ptr([&](ptr_t ptr) { battery_percent = ptr->battery_percent(); });
}

handler_t
STD_CALL
autolabor::pm1::native::
initialize_c(const char *port,
             double *progress) noexcept {
    const static auto serial_ports = [] {
        auto                     info = serial::list_ports();
        std::vector<std::string> result(info.size());
        std::transform(info.begin(), info.end(), result.begin(),
                       [](const serial::PortInfo &it) { return it.port; });
        return result;
    };
    
    handler_t id = ++task_id;
    
    auto list = port == nullptr || std::strlen(port) == 0
                ? serial_ports()
                : std::vector<std::string>{std::string(port)};
    
    if (list.empty())
        exceptions.set(id, "no available port");
    else {
        std::stringstream builder;
        
        for (auto i = list.begin();;) {
            *progress = static_cast<double>(i - list.begin()) / list.size();
            try {
                #ifdef __GNUC__
                const static std::string except = "/dev/ttyS";
                if (list.size() == 1 && i->substr(0, except.size()) == except) throw std::logic_error("skip ttyS.");
                #endif
                auto ptr = std::make_shared<chassis>(*i);
                
                chassis_ptr(ptr);
                builder.str("");
                odometry_mark  = ptr->odometry().value;
                connected_port = *i;
                pause_flag     = false;
                cancel_flag    = false;
                break;
            }
            catch (std::exception &e) {
                builder << *i << " : " << e.what();
                if (++i < list.end())
                    builder << std::endl;
                else
                    break;
            }
        }
        exceptions.set(id, builder.str());
    }
    
    *progress = 1;
    return id;
}

handler_t
STD_CALL
autolabor::pm1::native::
initialize(const char *port,
           double &progress) noexcept {
    return initialize_c(port, &progress);
}

handler_t
STD_CALL
autolabor::pm1::native::
shutdown() noexcept {
    handler_t id = ++task_id;
    if (!chassis_ptr(nullptr))
        exceptions.set(id, "null chassis pointer");
    connected_port.clear();
    return id;
}

handler_t
STD_CALL
autolabor::pm1::native::
get_rudder_c(double *rudder) noexcept {
    return get_rudder(*rudder);
}

handler_t
STD_CALL
autolabor::pm1::native::
get_rudder(double &rudder) noexcept {
    handler_t id = ++task_id;
    try {
        rudder = chassis_ptr.read<double>([&](ptr_t ptr) {
            return ptr->rudder().position;
        });
    } catch (std::exception &e) {
        rudder = NAN;
        exceptions.set(id, e.what());
    }
    return id;
}

handler_t
STD_CALL
autolabor::pm1::native::
get_odometry_c(double *stamp,
               double *s, double *sa,
               double *x, double *y, double *theta) noexcept {
    return get_odometry(*stamp, *s, *sa,
                        *x, *y, *theta);
}

handler_t
STD_CALL
autolabor::pm1::native::
get_odometry(double &stamp,
             double &s, double &a,
             double &x, double &y, double &theta) noexcept {
    handler_t id = ++task_id;
    try {
        chassis_ptr.read<void>([&](ptr_t ptr) {
            auto value = ptr->odometry();
            auto temp  = value.value - odometry_mark;
            stamp = duration_seconds<>(value.time.time_since_epoch());
            s     = temp.s;
            a     = temp.a;
            x     = temp.x;
            y     = temp.y;
            theta = temp.theta;
        });
    }
    catch (std::exception &e) {
        stamp = NAN;
        s     = NAN;
        a     = NAN;
        x     = NAN;
        y     = NAN;
        theta = NAN;
        exceptions.set(id, e.what());
    }
    return id;
}

handler_t
STD_CALL
autolabor::pm1::native::
reset_odometry() noexcept {
    return use_ptr([](ptr_t ptr) {
        odometry_mark = ptr->odometry().value;
    });
}

handler_t
STD_CALL
autolabor::pm1::native::
set_command_enabled(bool value) noexcept {
    return use_ptr([value](ptr_t ptr) {
        ptr->command_enabled = value;
    });
}

handler_t
STD_CALL
autolabor::pm1::native::
set_enabled(bool value) noexcept {
    return use_ptr([value](ptr_t ptr) {
        ptr->set_enabled_target(value);
    });
}

unsigned char
STD_CALL
autolabor::pm1::native::
check_state() noexcept {
    try {
        return chassis_ptr.read<unsigned char>([](ptr_t ptr) {
            auto states = ptr->state().states;
            auto unique = std::unordered_set<node_state_t>(states.begin(), states.end());
            return 1 == unique.size()
                   ? static_cast<unsigned char>(*unique.cbegin())
                   : 0x7f;
        });
    } catch (std::exception &) {
        return 0;
    }
}

handler_t
STD_CALL
autolabor::pm1::native::
drive_physical(double speed, double rudder) noexcept {
    handler_t id = ++task_id;
    
    weak_lock_guard<decltype(action_mutex)> lock(action_mutex);
    if (!lock) {
        exceptions.set(id, action_conflict);
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
STD_CALL
autolabor::pm1::native::
drive_wheels(double left, double right) noexcept {
    handler_t id = ++task_id;
    
    weak_lock_guard<decltype(action_mutex)> lock(action_mutex);
    if (!lock) {
        exceptions.set(id, action_conflict);
        return id;
    }
    
    try {
        chassis_ptr.read<void>([=](ptr_t ptr) {
            auto physical = wheels_to_physical(wheels{static_cast<float>(left),
                                                      static_cast<float>(right)},
                                               &ptr->config);
            ptr->set_target(physical.speed, physical.rudder);
        });
    } catch (std::exception &e) {
        exceptions.set(id, e.what());
    }
    return id;
}

handler_t
STD_CALL
autolabor::pm1::native::
drive_velocity(double v, double w) noexcept {
    handler_t id = ++task_id;
    
    weak_lock_guard<decltype(action_mutex)> lock(action_mutex);
    if (!lock) {
        exceptions.set(id, action_conflict);
        return id;
    }
    
    try {
        chassis_ptr.read<void>([=](ptr_t ptr) {
            auto physical = velocity_to_physical(velocity{static_cast<float>(v),
                                                          static_cast<float>(w)},
                                                 &ptr->config);
            ptr->set_target(physical.speed, physical.rudder);
        });
    } catch (std::exception &e) {
        exceptions.set(id, e.what());
    }
    return id;
}

handler_t block(double v,
                double w,
                double limit,
                const autolabor::process_controller &controller,
                std::function<double(ptr_t)> &&measure,
                double &progress) noexcept {
    handler_t id = ++task_id;
    progress = 0;
    
    weak_lock_guard<decltype(action_mutex)> lock(action_mutex);
    if (!lock) {
        exceptions.set(id, action_conflict);
        return id;
    }
    
    auto rest   = 1 - progress;
    auto paused = true;
    try {
        auto config = chassis_ptr.read<chassis_config_t>([](ptr_t ptr) { return ptr->config; });
        auto target = velocity_to_physical(velocity{static_cast<float>(v),
                                                    static_cast<float>(w)},
                                           &config);
        
        autolabor::process_t process{0, 0, target.speed};
        
        while (true) {
            if (cancel_flag) {
                chassis_ptr.read<void>([](ptr_t ptr) { ptr->set_target(0, NAN); });
                throw std::logic_error("action canceled");
            }
            
            if (paused) {
                // 检查恢复标记
                if (!(paused = pause_flag)) {
                    process.begin = chassis_ptr.read(measure);
                    process.end   = process.begin + limit;
                }
            } else {
                auto finished = chassis_ptr.read<bool>([&](ptr_t ptr) {
                    constexpr static auto
                        unknown  = autolabor::pm1::node_state_t::unknown,
                        enabled  = autolabor::pm1::node_state_t::enabled,
                        disabled = autolabor::pm1::node_state_t::disabled;
                    
                    // 检查状态
                    auto states = ptr->state().states;
                    if (std::find(states.begin(), states.end(), unknown) != states.end()) {
                        std::stringstream builder;
                        builder << "critical error: error state -> [ecu0|ecu1|tcu|vcu] = ["
                                << static_cast<int>(states[0]) << '|'
                                << static_cast<int>(states[1]) << '|'
                                << static_cast<int>(states[2]) << '|'
                                << static_cast<int>(states[3]) << ']';
                        throw std::logic_error(builder.str());
                    }
                    if (std::find(states.begin(), states.end(), disabled) != states.end()
                        && ptr->target_state() != enabled)
                        throw std::logic_error("chassis is locked");
                    
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
                
                if (finished)
                    break;
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
STD_CALL
autolabor::pm1::native::
calculate_spatium(double spatium, double angle, double width) noexcept {
    return std::abs(spatium + width / 2 * angle) +
           std::abs(spatium - width / 2 * angle);
}

handler_t
STD_CALL
autolabor::pm1::native::
drive_spatial(double v,
              double w,
              double spatium,
              double angle,
              double &progress) noexcept {
    odometry_t origin{};
    double     width;
    
    try {
        origin = chassis_ptr.read<odometry_t<>>([](ptr_t ptr) { return ptr->odometry().value; });
        width  = chassis_ptr.read<double>([](ptr_t ptr) { return ptr->config.width; });
    } catch (std::exception &e) {
        handler_t id = ++task_id;
        exceptions.set(id, e.what());
        return id;
    }
    
    return block(v, w, calculate_spatium(spatium, angle, width),
                 {0.5, 0.1, 12, 4},
                 [origin, width](ptr_t ptr) {
                     auto odometry = ptr->odometry().value - origin;
                     return calculate_spatium(odometry.s, odometry.a, width);
                 },
                 progress);
}

handler_t
STD_CALL
autolabor::pm1::native::
drive_spatial_c(double v,
                double w,
                double spatium,
                double angle,
                double *progress) noexcept {
    return drive_spatial(v, w, spatium, angle, *progress);
}

handler_t
STD_CALL
autolabor::pm1::native::
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
STD_CALL
autolabor::pm1::native::
adjust_rudder(double offset,
              double &progress) noexcept {
    handler_t id = ++task_id;
    
    weak_lock_guard<decltype(action_mutex)> lock(action_mutex);
    if (!lock) {
        exceptions.set(id, action_conflict);
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
                throw std::logic_error("action canceled");
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
                
                if (finished)
                    break;
            }
            std::this_thread::sleep_for(50ms);
        }
    } catch (std::exception &e) {
        exceptions.set(id, e.what());
    }
    
    return id;
}

void
STD_CALL
autolabor::pm1::native::
set_paused(bool paused) noexcept { pause_flag = paused; }

bool
STD_CALL
autolabor::pm1::native::
is_paused() noexcept { return pause_flag; }

void
STD_CALL
autolabor::pm1::native::
cancel_action() noexcept {
    cancel_flag = true;
    {
        std::lock_guard<std::mutex> wait(action_mutex);
    }
    cancel_flag = false;
}

#pragma clang diagnostic pop
