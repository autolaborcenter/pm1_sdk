//
// Created by ydrml on 2019/2/25.
//

#include "chassis.hh"

#include <cmath>
#include <condition_variable>

#include <utilities/serial_parser/parse_engine.hpp>
#include <utilities/differentiator_t.hpp>

#include "can/parser_t.hpp"
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
inline void atomic_plus_assign_stamped(
    std::atomic<autolabor::stamped_t<t1>> &a,
    const autolabor::stamped_t<t2> &b
) noexcept {
    autolabor::stamped_t<t1>
        expected = a.load(),
        desired{b.time, expected.value + b.value};
    while (!a.compare_exchange_strong(expected, desired))
        desired = {b.time, expected.value + b.value};
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
#include <iostream>
#include <utilities/differentiator_t.hpp>

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
    delay_interval      = std::chrono::milliseconds(max_of(5, timeout_gcd - 1));

struct wheel_mark_t {
    unsigned long seq;
    double        last,
                  current;
};

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
    
    using result_t = can::parser_t::result_type_t;
    using engine_t = parse_engine_t<can::parser_t>;
    
    port << can::pack<ecu<>::timeout>({2, 0}) // 设置动力超时时间到 200 ms
         << can::pack<unit<>::emergency_stop>();          // 从锁定状态启动
    
    start_write_loop();
    
    // region check nodes
    {
        std::condition_variable signal;
    
        volatile bool temp[]{false, false, false};
        
        auto timer = std::thread([&] {
            using namespace std::chrono_literals;
    
            std::mutex                   signal_mutex;
            std::unique_lock<std::mutex> own(signal_mutex);
            if (signal.wait_for(own, check_timeout, [&] { return !running || (temp[0] && temp[1] && temp[2]); }))
                return;
    
            stop_all();
        });
    
        auto parse = [&](const autolabor::can::parser_t::result_t &result) {
            if (result.type != result_t::message) return;
        
            const auto _now = now();
        
            switch (_odometry.try_parse(_now, result.message, config)) {
                case pm1_odometry_t::result_type::left:
                    temp[0] = true;
                    break;
                case pm1_odometry_t::result_type::right:
                    temp[1] = true;
                    break;
                case pm1_odometry_t::result_type::none: {
                    const auto last  = _rudder;
                    const auto value = RAD_OF(get_data_value<int>(result.message), default_wheel_k);
                    _rudder = {_now, {value, value - last.value.position / duration_seconds(_now - last.time)}};
                }
                    temp[2] = true;
                    break;
            }
        };
    
        engine_t engine;
        uint8_t  buffer[64];
        while (!temp[0] || !temp[1] || !temp[2]) {
            try { engine(buffer, buffer + port.read(buffer, sizeof(buffer)), parse); }
            catch (...) { stop_all(); }
            if (!running) {
                running = false;
                std::stringstream builder;
                builder << "it's not a pm1 chassis: [ecu0|ecu1|tcu0] = ["
                        << (temp[0] ? '*' : 'x') << '|'
                        << (temp[1] ? '*' : 'x') << '|'
                        << (temp[2] ? '*' : 'x') << ']';
                timer.join();
                write_thread.join();
                throw std::runtime_error(builder.str());
            }
        }
        signal.notify_all();
        timer.join();
    }
    // endregion
    // region receive
    read_thread = std::thread([=] {
        const auto t0    = now();
        auto       speed = .0f;
    
        std::array<decltype(now()), 4> reply_time{t0, t0, t0, t0};
        
        auto parse = [&](const autolabor::can::parser_t::result_t &result) {
            if (result.type != result_t::message) return;
            
            auto _now = now();
            
            for (size_t i = 0; i < reply_time.size(); ++i)
                if (_now - reply_time[i] > state_timeout)
                    chassis_state.states[i] = node_state_t::unknown;
            
            auto msg = result.message;
    
            if (unit<ecu<0>>::state_rx::match(msg)) {
                reply_time[0] = _now;
                if (node_state_t::enabled == (chassis_state.ecu0() = parse_state(*msg.data))) {
                    if (!enabled_target)
                        port << can::pack<unit<ecu<0>>::emergency_stop>();
                } else {
                    if (enabled_target)
                        port << pack_value<unit<ecu<0>>::release_stop, uint8_t>(0xff);
                }
        
            } else if (unit<ecu<1>>::state_rx::match(msg)) {
                reply_time[1] = _now;
                if (node_state_t::enabled == (chassis_state.ecu1() = parse_state(*msg.data))) {
                    if (!enabled_target)
                        port << can::pack<unit<ecu<1>>::emergency_stop>();
                } else {
                    if (enabled_target)
                        port << pack_value<unit<ecu<1>>::release_stop, uint8_t>(0xff);
                }
        
            } else if (unit<tcu<0>>::state_rx::match(msg)) {
                reply_time[2] = _now;
                if (node_state_t::enabled == (chassis_state.tcu() = parse_state(*msg.data))) {
                    if (!enabled_target)
                        port << can::pack<unit<tcu<0>>::emergency_stop>();
                } else {
                    if (enabled_target)
                        port << pack_value<unit<tcu<0>>::release_stop, uint8_t>(0xff);
                }
        
            } else if (unit<vcu<0>>::state_rx::match(msg)) {
                reply_time[3] = _now;
                chassis_state.vcu() = parse_state(*msg.data);
        
            } else if (vcu<0>::battery_percent_rx::match(msg)) {
                _battery = *msg.data;
        
            } else if (_odometry.try_parse(_now, msg, config) == pm1_odometry_t::result_type::none) {
            
            } else if (tcu<0>::current_position_rx::match(msg)) {
        
                const auto last  = _rudder;
                const auto value = RAD_OF(get_data_value<int>(result.message), default_wheel_k);
                _rudder = {_now, {value, value - last.value.position / duration_seconds(_now - last.time)}};
                
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
        while (running)
            try {
                engine(buffer, buffer + port.read(buffer, sizeof(buffer)), parse);
            } catch (...) {
                stop_all();
            }
    });
    // endregion
    // region wait state
    const auto end = now() + state_interval + check_state_timeout;
    do {
        auto &states = chassis_state.states;
        if (states.end() == std::find(states.begin(),
                                      states.end(),
                                      node_state_t::unknown))
            break;
        std::this_thread::sleep_for(5ms);
    } while (now() < end);
    // endregion
}

chassis::~chassis() {
    stop_all();
    read_thread.join();
    write_thread.join();
}

//==============================================================

autolabor::pm1::motor_t chassis::left() const {
    return _odometry._left.value;
}

autolabor::pm1::motor_t chassis::right() const {
    return _odometry._right.value;
}

autolabor::pm1::motor_t chassis::rudder() const {
    return _rudder.value;
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

autolabor::stamped_t<autolabor::odometry_t<>>
chassis::odometry() const {
    return _odometry.value();
}

double chassis::battery_percent() const {
    return _battery / 100.0;
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
    write_thread = std::thread([this] {
        using t = decltype(now());
    
        differentiator_t<t>
            odometry_time{{}, [](const t &t0, const t &t1) { return t1 - t0 > odometry_interval; }},
            rudder_time{{}, [](const t &t0, const t &t1) { return t1 - t0 > rudder_interval; }},
            state_time{{}, [](const t &t0, const t &t1) { return t1 - t0 > state_interval; }};
    
        std::mutex lock;
        
        do {
            auto _now = now();
            t    _;
    
            if (odometry_time.update(_now, _))
                _odometry.ask(port);
    
            if (rudder_time.update(_now, _))
                port << autolabor::can::pack<tcu<0>::current_position_tx>();
    
            if (state_time.update(_now, _)) {
                port << autolabor::can::pack<unit<>::state_tx>()
                     << autolabor::can::pack<vcu<>::battery_percent_tx>();
                AVOID_SLEEP;
            }
    
            std::unique_lock<decltype(lock)> _lk(lock);
            if (synchronizer.wait_for(_lk, delay_interval, [this] { return !running; }))
                return;
        } while (running);
    });
}

void chassis::stop_all() {
    running = false;
    synchronizer.notify_all();
    port.break_read();
}
