//
// Created by ydrml on 2019/2/25.
//

#include "chassis.hh"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <condition_variable>

#include "can/parser_t.hpp"
#include "serial_parser/parse_engine.hpp"
#include "odometry.h"

extern "C" {
#include "control_model/motor_map.h"
#include "control_model/optimization.h"
}

using namespace autolabor::pm1;

// region functions

template<class t>
inline serial_port &operator<<(
    serial_port &port,
    const t &msg) noexcept {
    port.send(bytes_begin(msg), sizeof(t));
    return port;
}

template<class t1, class t2>
inline void atomic_plus_assign(std::atomic<t1> &a, const t2 &b) noexcept {
    auto expected = a.load();
    auto desired  = expected + b;
    while (!a.compare_exchange_strong(expected, desired))
        desired = expected + b;
}

constexpr uint64_t max_of(uint64_t a, uint64_t b) noexcept {
    return a > b ? a : b;
}

constexpr uint64_t gcd(uint64_t a, uint64_t b) noexcept {
    return a <= 1 || b <= 1 ? 1 : a > b ? gcd(b, a % b) : gcd(a, b % a);
}

template<class t>
constexpr uint64_t count_ms(t time) noexcept {
    return std::chrono::duration_cast<std::chrono::milliseconds>(time).count();
}

// endregion

const float
    chassis::default_max_wheel_speed = 1.1f,
    chassis::default_max_v           = default_max_wheel_speed,
    chassis::default_max_w           = pi_f / 4,
    chassis::default_optimize_width  = pi_f / 4,
    chassis::default_acceleration    = 10;

#if   defined(WIN32)

#include <Windows.h>

#define AVOID_SLEEP SetThreadExecutionState(ES_DISPLAY_REQUIRED | ES_SYSTEM_REQUIRED)
constexpr auto timeout = 3;
#elif defined(linux)
#define AVOID_SLEEP
constexpr auto timeout = 20;
#else
#error unsupported platform
#endif

using namespace std::chrono_literals;

constexpr auto
    odometry_interval   = 50ms,
    rudder_interval     = 20ms,
    state_interval      = 1000ms,
    state_timeout       = state_interval + 100ms,
    control_timeout     = 500ms,
    check_timeout       = 1000ms,
    check_state_timeout = 100ms;
constexpr auto
    timeout_gcd         = gcd(count_ms(state_interval),
                              gcd(count_ms(odometry_interval),
                                  count_ms(rudder_interval)));
constexpr auto
    delay_interval      = std::chrono::milliseconds(max_of(1, timeout_gcd - 1));

chassis::chassis(const std::string &port_name)
    : port(port_name, 115200, timeout),
      running(std::make_shared<bool>(true)),
      command_enabled(true),
      config(default_config),
      max_v(default_max_v),
      max_w(default_max_w),
      max_wheel_speed(default_max_wheel_speed),
      optimize_width(default_optimize_width),
      acceleration(default_acceleration),
      enabled_target(false) {
    
    using result_t = can::parser_t::result_type_t;
    using engine_t = parse_engine_t<can::parser_t>;
    
    _left.time = _right.time = _rudder.time = now();
    
    port << can::pack<ecu<>::timeout>({2, 0}) // 设置动力超时时间到 200 ms
         << can::pack<unit<>::emergency_stop>();          // 从锁定状态启动
    
    start_write_loop();
    
    // region check nodes
    {
        std::condition_variable signal;
    
        volatile bool temp[]{false, false, false},
                      abandon = false;
    
        auto timer = std::thread([&] {
            using namespace std::chrono_literals;
        
            std::mutex                   signal_mutex;
            std::unique_lock<std::mutex> own(signal_mutex);
            if (signal.wait_for(own, check_timeout, [&] { return temp[0] && temp[1] && temp[2]; }))
                return;
        
            abandon = true;
            port.break_read();
        });
    
        auto parse = [&](const autolabor::can::parser_t::result_t &result) {
            if (result.type != result_t::message) return;
        
            if (ecu<0>::current_position_rx::match(result.message)) {
                _left.update(now(), RAD_OF(get_data_value<int>(result.message), default_wheel_k));
                temp[0] = true;
            } else if (ecu<1>::current_position_rx::match(result.message)) {
                _right.update(now(), RAD_OF(get_data_value<int>(result.message), default_wheel_k));
                temp[1] = true;
            } else if (tcu<0>::current_position_rx::match(result.message)) {
                _rudder.update(now(), RAD_OF(get_data_value<short>(result.message), default_rudder_k));
                temp[2] = true;
            }
        };
    
        engine_t engine;
        uint8_t  buffer[64];
        while (!temp[0] || !temp[1] || !temp[2]) {
            try { engine(buffer, buffer + port.read(buffer, sizeof(buffer)), parse); }
            catch (...) { abandon = true; }
            if (abandon) {
                *running = false;
                std::stringstream builder;
                builder << "it's not a pm1 chassis: [ecu0|ecu1|tcu0] = ["
                        << (temp[0] ? '*' : 'x') << '|'
                        << (temp[1] ? '*' : 'x') << '|'
                        << (temp[2] ? '*' : 'x') << ']';
                timer.join();
                throw std::runtime_error(builder.str());
            }
        }
        signal.notify_all();
        timer.join();
    }
    // endregion
    // region receive
    read_thread = std::thread([=] {
        auto left_ready  = false,
             right_ready = false;
        auto delta_left  = .0,
             delta_right = .0;
        auto time        = now();
        auto speed       = .0f;
        
        std::array<decltype(now()), 4> reply_time{time, time, time, time};
        
        auto parse = [&](const autolabor::can::parser_t::result_t &result) {
            if (result.type != result_t::message) return;
            
            auto _now = now();
            
            for (size_t i = 0; i < reply_time.size(); ++i)
                if (_now - reply_time[i] > state_timeout)
                    chassis_state.states[i] = node_state_t::unknown;
            
            auto msg = result.message;
            
            if (unit<ecu<0 >>::state_rx::match(msg)) {
                reply_time[0] = _now;
                if (node_state_t::enabled == (chassis_state.ecu0() = parse_state(*msg.data))) {
                    if (!enabled_target)
                        port << can::pack<unit<ecu<0 >>::emergency_stop>();
                } else {
                    if (enabled_target)
                        port << pack_value<unit<ecu<0 >>::release_stop, uint8_t>(0xff);
                }
                
            } else if (unit<ecu<1 >>::state_rx::match(msg)) {
                reply_time[1] = _now;
                if (node_state_t::enabled == (chassis_state.ecu1() = parse_state(*msg.data))) {
                    if (!enabled_target)
                        port << can::pack<unit<ecu<1 >>::emergency_stop>();
                } else {
                    if (enabled_target)
                        port << pack_value<unit<ecu<1 >>::release_stop, uint8_t>(0xff);
                }
                
            } else if (unit<tcu<0 >>::state_rx::match(msg)) {
                reply_time[2] = _now;
                if (node_state_t::enabled == (chassis_state.tcu() = parse_state(*msg.data))) {
                    if (!enabled_target)
                        port << can::pack<unit<tcu<0 >>::emergency_stop>();
                } else {
                    if (enabled_target)
                        port << pack_value<unit<tcu<0 >>::release_stop, uint8_t>(0xff);
                }
                
            } else if (unit<vcu<0 >>::state_rx::match(msg)) {
                reply_time[3] = _now;
                chassis_state.vcu() = parse_state(*msg.data);
                
            } else if (ecu<0>::current_position_rx::match(msg)) {
                
                auto value = RAD_OF(get_data_value<int>(msg), default_wheel_k);
                delta_left = value - _left.position;
                
                _left.update(_now, value);
                
                if (right_ready) {
                    atomic_plus_assign(_odometry,
                                       wheels_to_odometry(
                                           delta_left, delta_right, config
                                       ));
                    right_ready = false;
                    time        = _now;
                } else
                    left_ready = true;
                
            } else if (ecu<1>::current_position_rx::match(msg)) {
                
                auto value = RAD_OF(get_data_value<int>(msg), default_wheel_k);
                delta_right = value - _right.position;
                
                _right.update(_now, value);
                
                if (left_ready) {
                    atomic_plus_assign(_odometry,
                                       wheels_to_odometry(
                                           delta_left, delta_right, config
                                       ));
                    left_ready = false;
                    time       = _now;
                } else
                    right_ready = true;
                
            } else if (tcu<0>::current_position_rx::match(msg)) {
                
                auto value = RAD_OF(get_data_value<short>(msg), default_rudder_k);
                _rudder.update(_now, value);
                
                if (std::isnan(target.rudder) || now() - request_time > control_timeout)
                    target = {0, value};
                
                constexpr static auto period = duration_seconds<float>(rudder_interval);
                
                auto optimized = optimize(target, {speed, value},
                                          optimize_width, acceleration * period);
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
        };
        
        engine_t engine;
        uint8_t  buffer[64];
        while (*running)
            try {
                engine(buffer, buffer + port.read(buffer, sizeof(buffer)), parse);
            } catch (...) {
                *running = false;
            }
    });
    // endregion
    // region wait state
    for (const auto time = now();
         now() - time < check_state_timeout;) {
        auto &states = chassis_state.states;
        if (states.end() == std::find(states.begin(),
                                      states.end(),
                                      node_state_t::unknown))
            break;
        std::this_thread::sleep_for(5ms);
    }
    // endregion
}

chassis::~chassis() {
    *running = false;
    port.break_read();
    read_thread.detach();
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

autolabor::odometry_t<> chassis::odometry() const {
    return _odometry;
}

bool chassis::is_threads_running() const {
    return running && *running;
}

//==============================================================

void chassis::set_enabled_target(bool state) {
    (enabled_target = state)
    ? port << pack_value<unit<>::release_stop, uint8_t>(0xff)
    : port << can::pack<unit<>::emergency_stop>();
}

void chassis::set_target(double speed, double rudder) {
    std::lock_guard<decltype(target_mutex)> l1(target_mutex);
    
    request_time = now();
    target       = {static_cast<float>(speed), static_cast<float>(rudder)};
    limit_in_velocity(&target, max_v, max_w, &config);
    limit_in_physical(&target, max_wheel_speed);
}

void chassis::reset_rudder() {
    port << autolabor::can::pack<tcu<0>::encoder_reset>();
    target.rudder = 0;
}

void chassis::start_write_loop() {
    std::thread([flag = running, this] {
        decltype(now()) _now, task_time[3]{};
        
        do {
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
        } while (*flag);
    }).detach();
}
