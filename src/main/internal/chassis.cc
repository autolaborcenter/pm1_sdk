//
// Created by ydrml on 2019/2/25.
//

#include "chassis.hh"

#include <algorithm>
#include <cmath>
#include <sstream>
#include "can/parse_engine.hh"
#include "raii/weak_shared_lock.hpp"

extern "C" {
#include "control_model/motor_map.h"
#include "control_model/optimization.h"
}

#ifdef _MSC_VER

#include <Windows.h>

#define AVOID_SLEEP SetThreadExecutionState(ES_DISPLAY_REQUIRED | ES_SYSTEM_REQUIRED)
#else
#define AVOID_SLEEP
#endif

using namespace autolabor::pm1;

// region functions

template<class t>
inline serial_port &operator<<(
    serial_port &port,
    const autolabor::can::msg_union<t> &msg) noexcept {
    port.send(msg.bytes, sizeof(t));
    return port;
}

template<class t>
inline void atomic_plus_assign(std::atomic<t> &a, const t &b) noexcept {
    auto expected = a.load();
    auto desired  = expected + b;
    while (!a.compare_exchange_strong(expected, desired))
        desired = expected + b;
}

constexpr unsigned long max_of(unsigned long a, unsigned long b) noexcept {
    return a > b ? a : b;
}

constexpr unsigned long gcd(unsigned long a, unsigned long b) noexcept {
    return a <= 1 || b <= 1 ? 1 : a > b ? gcd(b, a % b) : gcd(a, b % a);
}

template<class t>
constexpr unsigned long count_ms(t time) noexcept {
    return std::chrono::duration_cast<std::chrono::milliseconds>(time).count();
}

// endregion

const float
    chassis::default_max_wheel_speed = pi_f * 3.5f,
    chassis::default_max_v           = 1.1f,
    chassis::default_max_w           = pi_f / 4,
    chassis::default_optimize_width  = pi_f / 4,
    chassis::default_acceleration    = default_max_wheel_speed;

#if   defined(WIN32)
constexpr auto timeout = 3;
#elif defined(linux)
constexpr auto timeout = 20;
#else
#error unsupported platform
#endif

using namespace std::chrono_literals;

constexpr auto
    odometry_interval   = 50ms,
    rudder_interval     = 20ms,
    state_interval      = 1000ms,
    state_timeout       = 1100ms,
    control_timeout     = 500ms,
    check_timeout       = 1000ms,
    check_state_timeout = 100ms;
constexpr auto
    timeout_gcd         = gcd(count_ms(state_interval),
                              gcd(count_ms(odometry_interval),
                                  count_ms(rudder_interval)));
constexpr auto
    delay_interval      = std::chrono::milliseconds(max_of(1, timeout_gcd - 1));
constexpr auto
    frequency           = 1000.0f / count_ms(rudder_interval);

chassis::chassis(const std::string &port_name)
    : port(port_name, 115200, timeout),
      running(true),
      command_enabled(true),
      config(default_config),
      max_v(default_max_v),
      max_w(default_max_w),
      max_wheel_speed(default_max_wheel_speed),
      optimize_width(default_optimize_width),
      acceleration(default_acceleration),
      enabled_target(false) {
    
    using result_t  = autolabor::can::parser::result_type;
    
    _left.time = _right.time = _rudder.time = now();
    
    // region check nodes
    {
        port << autolabor::can::pack<ecu<>::current_position_tx>()
             << autolabor::can::pack<tcu<0>::current_position_tx>();
        
        const auto time = now();
        bool       temp[]{false, false, false};
        
        auto is_timeout = [time] { return now() - time > check_timeout; };
        auto done       = [&temp] { return temp[0] && temp[1] && temp[2]; };
        
        auto task = std::thread([&] {
            while (!done()) {
                if (is_timeout()) {
                    port.break_read();
                    return;
                }
                std::this_thread::sleep_for(check_timeout / 20);
            }
        });
        
        autolabor::can::parse_engine parser(
            [&, this](const autolabor::can::parser::result &result) {
                if (result.type != result_t::message) return;
                
                auto _now = now();
                auto msg  = result.message;
                
                if (ecu<0>::current_position_rx::match(msg)) {
                    _left.update(_now, RAD_OF(get_data_value<int>(msg), default_wheel_k));
                    temp[0] = true;
                } else if (ecu<1>::current_position_rx::match(msg)) {
                    _right.update(_now, RAD_OF(get_data_value<int>(msg), default_wheel_k));
                    temp[1] = true;
                } else if (tcu<0>::current_position_rx::match(msg)) {
                    _rudder.update(_now, RAD_OF(get_data_value<short>(msg), default_rudder_k));
                    temp[2] = true;
                }
            });
        
        uint8_t buffer[64];
        while (!done()) {
            auto actual = port.read(buffer, sizeof(buffer));
            
            for (size_t i = 0; i < actual; ++i)
                parser(buffer[i]);
            
            if (is_timeout()) {
                std::stringstream builder;
                builder << "it's not a pm1 chassis: [ecu0|ecu1|tcu0] = ["
                        << static_cast<int>(temp[0]) << '|'
                        << static_cast<int>(temp[1]) << '|'
                        << static_cast<int>(temp[2]) << ']';
                
                task.join();
                throw std::runtime_error(builder.str());
            }
        }
        task.join();
    }
    // endregion
    // region initialize ask
    port << can::pack<ecu<>::timeout>({2, 0}) // 设置动力超时时间到 200 ms
         << can::pack<unit<>::emergency_stop>()           // 从锁定状态启动
         << can::pack<unit<>::state_tx>();                // 询问状态
    // endregion
    // region ask
    write_thread = std::thread([this] {
        auto           _now        = now();
        decltype(_now) task_time[] = {_now, _now, _now};
        
        while (running) {
            _now = now();
            
            if (_now - task_time[0] > odometry_interval) {
                port << autolabor::can::pack<ecu<>::current_position_tx>();
                task_time[0] = _now;
            }
            if (_now - task_time[1] > rudder_interval) {
                port << autolabor::can::pack<tcu<0>::current_position_tx>();
                task_time[1] = _now;
            }
            if (_now - task_time[2] > state_interval) {
                AVOID_SLEEP;
                port << autolabor::can::pack<unit<>::state_tx>();
                task_time[2] = _now;
            }
            
            std::this_thread::sleep_for(delay_interval);
        }
    });
    // endregion
    // region receive
    read_thread  = std::thread([=] {
        auto left_ready  = false,
             right_ready = false;
        auto delta_left  = .0,
             delta_right = .0;
        auto time        = now();
        auto speed       = .0f;
    
        std::array<decltype(now()), 4> reply_time{time, time, time, time};
        
        autolabor::can::parse_engine parser(
            [&](const autolabor::can::parser::result &result) {
                if (result.type != result_t::message) return;
                
                auto _now = now();
    
                for (size_t i = 0; i < reply_time.size(); ++i)
                    if (_now - reply_time[i] > state_timeout)
                        chassis_state.states[i] = node_state_t::unknown;
                
                // 处理
                const auto msg = result.message;
                
                if (unit<ecu<0>>::state_rx::match(msg)) {
                    reply_time[0] = _now;
                    if (node_state_t::enabled == (chassis_state.ecu0() = parse_state(*msg.data.data))) {
                        if (!enabled_target)
                            port << can::pack<unit<ecu<0>>::emergency_stop>();
                    } else {
                        if (enabled_target)
                            port << pack_value<unit<ecu<0>>::release_stop, uint8_t>(0xff);
                    }
                    
                } else if (unit<ecu<1>>::state_rx::match(msg)) {
                    reply_time[1] = _now;
                    if (node_state_t::enabled == (chassis_state.ecu1() = parse_state(*msg.data.data))) {
                        if (!enabled_target)
                            port << can::pack<unit<ecu<1>>::emergency_stop>();
                    } else {
                        if (enabled_target)
                            port << pack_value<unit<ecu<1>>::release_stop, uint8_t>(0xff);
                    }
                    
                } else if (unit<tcu<0>>::state_rx::match(msg)) {
                    reply_time[2] = _now;
                    if (node_state_t::enabled == (chassis_state.tcu() = parse_state(*msg.data.data))) {
                        if (!enabled_target)
                            port << can::pack<unit<tcu<0>>::emergency_stop>();
                    } else {
                        if (enabled_target)
                            port << pack_value<unit<tcu<0>>::release_stop, uint8_t>(0xff);
                    }
                    
                } else if (unit<vcu<0>>::state_rx::match(msg)) {
                    reply_time[3] = _now;
                    chassis_state.vcu() = parse_state(*msg.data.data);
                    
                } else if (ecu<0>::current_position_rx::match(msg)) {
                    
                    auto value = RAD_OF(get_data_value<int>(msg), default_wheel_k);
                    delta_left = value - _left.position;
                    
                    _left.update(_now, value);
                    
                    
                    if (right_ready) {
                        odometry_t delta = delta_differential_t{config.width,
                                                                config.radius * delta_left,
                                                                config.radius * delta_right,
                                                                _now - time};
                        atomic_plus_assign(_odometry, delta);
                        right_ready = false;
                        time        = _now;
                    } else
                        left_ready = true;
                    
                } else if (ecu<1>::current_position_rx::match(msg)) {
                    
                    auto value = RAD_OF(get_data_value<int>(msg), default_wheel_k);
                    delta_right = value - _right.position;
                    
                    _right.update(_now, value);
                    
                    if (left_ready) {
                        odometry_t delta = delta_differential_t{config.width,
                                                                config.radius * delta_left,
                                                                config.radius * delta_right,
                                                                _now - time};
                        atomic_plus_assign(_odometry, delta);
                        left_ready = false;
                        time       = _now;
                    } else
                        right_ready = true;
                    
                } else if (tcu<0>::current_position_rx::match(msg)) {
                    
                    auto value = RAD_OF(get_data_value<short>(msg), default_rudder_k);
                    _rudder.update(_now, value);
                    
                    if (std::isnan(target.rudder) || now() - request_time > control_timeout)
                        target         = {0, value};
                    
                    physical current{speed, value};
                    auto     optimized = optimize(&target,
                                                  &current,
                                                  optimize_width,
                                                  acceleration / frequency);
                    speed = optimized.speed;
    
                    auto wheels = physical_to_wheels(optimized, &config);
                    auto left   = PULSES_OF(wheels.left, default_wheel_k);
                    auto right  = PULSES_OF(wheels.right, default_wheel_k);
                    auto rudder = static_cast<short>(PULSES_OF(target.rudder, default_rudder_k));
    
                    if (command_enabled)
                        port << pack_value<ecu<0>::target_speed, int>(left)
                             << pack_value<ecu<1>::target_speed, int>(right)
                             << pack_value<tcu<0>::target_position, short>(rudder);
                }
            });
        
        uint8_t buffer[64];
        while (running) {
            try {
                for (size_t actual = port.read(buffer, sizeof(buffer)), i = 0; i < actual; ++i)
                    parser(buffer[i]);
            } catch (std::exception &) {
                running = false;
            }
        }
    });
    // endregion
    // region wait state
    for (const auto time = now();
         now() - time < check_state_timeout;) {
        if (std::none_of(chassis_state.states.begin(), chassis_state.states.end(),
                         [](node_state_t it) { return it == node_state_t::unknown; }))
            break;
        std::this_thread::sleep_for(5ms);
    }
    // endregion
}

chassis::~chassis() {
    if (destruct_once.test_and_set())
        return;
    
    running = false;
    port.break_read();
    
    std::this_thread::sleep_for(3 * delay_interval);
    read_thread.join();
    write_thread.join();
}

//==============================================================

autolabor::motor_t<> chassis::left() const {
    return _left;
}

autolabor::motor_t<> chassis::right() const {
    return _right;
}

autolabor::motor_t<> chassis::rudder() const {
    return _rudder;
}

chassis_state_t chassis::state() const {
    return running
           ? chassis_state
           : chassis_state_t{};
}

node_state_t chassis::target_state() const {
    return running
           ? enabled_target
             ? node_state_t::enabled
             : node_state_t::disabled
           : node_state_t::unknown;
}

autolabor::odometry_t chassis::odometry() const {
    return _odometry;
}

bool chassis::is_threads_running() const {
    return running;
}

//==============================================================

void chassis::set_enabled_target(bool state) {
    (enabled_target = state)
    ? port << pack_value<unit<>::release_stop, uint8_t>(0xff)
    : port << can::pack<unit<>::emergency_stop>();
}

void chassis::set_target(double speed, double rudder) {
    weak_shared_lock l0(action_mutex);
    if (!l0) throw std::logic_error("an action is invoking.");
    
    std::lock_guard<decltype(target_mutex)> l1(target_mutex);
    
    request_time = now();
    target       = {static_cast<float>(speed), static_cast<float>(rudder)};
    limit_in_velocity(target, max_v, max_w, &config);
    limit_in_physical(target, max_wheel_speed);
}

void chassis::reset_rudder() {
    port << autolabor::can::pack<tcu<0>::encoder_reset>();
    target.rudder = 0;
}
