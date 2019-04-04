//
// Created by ydrml on 2019/2/25.
//

#include "chassis.hh"

#include <algorithm>
#include "serial_extension.h"
#include "can/parse_engine.hh"

extern "C" {
#include "control_model/motor_map.h"
#include "control_model/optimization.h"
}

using namespace autolabor::pm1;

bool chassis_state_t::check_all(node_state_t target) const {
	return _ecu0 == target && _ecu1 == target && _tcu == target;
}

template<class t>
inline void add(std::atomic<t> &a, const t &b) {
	auto expected = a.load();
	auto desired  = expected + b;
	while (!a.compare_exchange_strong(expected, desired))
		desired = expected + b;
}

constexpr unsigned long max(unsigned long a, unsigned long b) {
	return a > b ? a : b;
}

constexpr unsigned long gcd(unsigned long a, unsigned long b) {
	return a <= 1 || b <= 1 ? 1 : a > b ? gcd(b, a % b) : gcd(a, b % a);
}

chassis::chassis(const std::string &port_name,
                 const chassis_config_t &chassis_config,
                 float optimize_width,
                 float acceleration)
	: port(port_name, 115200),
	  running(true) {
	using result_t = autolabor::can::parser::result_type;
	
	constexpr static auto odometry_interval = std::chrono::milliseconds(50);
	constexpr static auto rudder_interval   = std::chrono::milliseconds(20);
	constexpr static auto state_interval    = std::chrono::milliseconds(1000);
	constexpr static auto state_timeout     = state_interval * 2;
	constexpr static auto control_timeout   = std::chrono::milliseconds(500);
	constexpr static auto check_timeout     = std::chrono::milliseconds(1000);
	
	_left.time = _right.time = _rudder.time = now();
	
	// region check nodes
	{
		port << autolabor::can::pack<ecu<>::current_position_tx>()
		     << autolabor::can::pack<tcu<0>::current_position_tx>()
		
		     << autolabor::can::pack<ecu<>::current_position_tx>()
		     << autolabor::can::pack<tcu<0>::current_position_tx>();
		
		const auto time = now();
		bool       temp[]{false, false, false};
		
		auto timeout = [time] { return now() - time > check_timeout; };
		auto done    = [&temp] { return temp[0] && temp[1] && temp[2]; };
		
		auto task = std::thread([&] {
			while (!done()) {
				if (timeout()) {
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
					_left.update(_now, RAD_OF(get_big_endian<int>(msg), default_wheel_k));
					temp[0] = true;
				} else if (ecu<1>::current_position_rx::match(msg)) {
					_right.update(_now, RAD_OF(get_big_endian<int>(msg), default_wheel_k));
					temp[1] = true;
				} else if (tcu<0>::current_position_rx::match(msg)) {
					_rudder.update(_now, RAD_OF(get_big_endian<short>(msg), default_rudder_k));
					temp[2] = true;
				}
			});
		
		uint8_t buffer[64];
		while (!done()) {
			auto actual = port.read(buffer, sizeof(buffer));
			
			for (auto i = 0; i < actual; ++i) parser(buffer[i]);
			
			if (timeout()) {
				task.join();
				throw std::exception("it's not a pm1 chassis");
			}
		}
		task.join();
	}
	// endregion
	
	port << autolabor::can::pack<ecu<>::timeout>({2, 0}) // 设置超时时间：200 ms
	     << autolabor::can::pack<unit<>::state_tx>();    // 询问状态
	
	// 定时任务
	read_thread = std::thread([this] {
		constexpr static auto gcd_ = gcd(state_interval.count(),
		                                 gcd(odometry_interval.count(),
		                                     rudder_interval.count()));
		constexpr static auto delay_interval
		                           = std::chrono::milliseconds(max(1, gcd_ - 1));
		
		auto           _now        = now();
		decltype(_now) task_time[] = {_now, _now, _now};
		
		while (running) {
			_now = now();
			
			try {
				if (_now - task_time[0] > odometry_interval) {
					port << autolabor::can::pack<ecu<>::current_position_tx>();
					task_time[0] = _now;
				}
				if (_now - task_time[1] > rudder_interval) {
					port << autolabor::can::pack<tcu<0>::current_position_tx>();
					task_time[1] = _now;
				}
				if (_now - task_time[2] > state_interval) {
					port << autolabor::can::pack<unit<>::state_tx>();
					task_time[2] = _now;
				}
			} catch (std::exception &e) { break; }
			
			std::this_thread::sleep_for(delay_interval);
		}
		
		if (running.exchange(false)) port.break_read();
	});
	
	// region receive
	write_thread = std::thread([=] {
		auto left_ready  = false,
		     right_ready = false;
		auto delta_left  = .0,
		     delta_right = .0;
		auto time        = now();
		auto speed       = .0f;
		
		decltype(time) reply_time[] = {time, time, time};
		
		autolabor::can::parse_engine parser(
			[&](const autolabor::can::parser::result &result) {
				if (result.type != result_t::message) return;
				
				auto _now = now();
				
				if (_now - reply_time[0] > state_timeout)
					chassis_state._ecu0 = node_state_t::unknown;
				
				if (_now - reply_time[1] > state_timeout)
					chassis_state._ecu1 = node_state_t::unknown;
				
				if (_now - reply_time[2] > state_timeout)
					chassis_state._tcu = node_state_t::unknown;
				
				// 处理
				const auto msg = result.message;
				
				if (unit<ecu<0 >>::state_rx::match(msg)) {
					chassis_state._ecu0 = parse_state(*msg.data.data);
					reply_time[0] = _now;
					
				} else if (unit<ecu<1 >>::state_rx::match(msg)) {
					chassis_state._ecu1 = parse_state(*msg.data.data);
					reply_time[1] = _now;
					
				} else if (unit<tcu<0 >>::state_rx::match(msg)) {
					chassis_state._tcu = parse_state(*msg.data.data);
					reply_time[2] = _now;
					
				} else if (ecu<0>::current_position_rx::match(msg)) {
					
					auto value = RAD_OF(get_big_endian<int>(msg), default_wheel_k);
					delta_left = value - _left.position;
					
					_left.update(_now, value);
					
					if (right_ready) {
						odometry_t delta = delta_differential_t{chassis_config.width,
						                                        chassis_config.radius * delta_left,
						                                        chassis_config.radius * delta_right,
						                                        _now - time};
						add(_odometry, delta);
						right_ready = false;
						time        = _now;
					} else
						left_ready = true;
					
				} else if (ecu<1>::current_position_rx::match(msg)) {
					
					auto value = RAD_OF(get_big_endian<int>(msg), default_wheel_k);
					delta_right = value - _right.position;
					
					_right.update(_now, value);
					
					if (left_ready) {
						odometry_t delta = delta_differential_t{chassis_config.width,
						                                        chassis_config.radius * delta_left,
						                                        chassis_config.radius * delta_right,
						                                        _now - time};
						add(_odometry, delta);
						left_ready = false;
						time       = _now;
					} else
						right_ready = true;
					
				} else if (tcu<0>::current_position_rx::match(msg)) {
					
					auto value = RAD_OF(get_big_endian<short>(msg), default_rudder_k);
					_rudder.update(_now, value);
					
					if (std::isnan(target.rudder) || now() - request_time > control_timeout)
						target         = {0, value};
					
					physical current{speed, value};
					auto     optimized = optimize(&target, &current, optimize_width, acceleration);
					auto     wheels    = physical_to_wheels(&optimized, &chassis_config);
					speed = optimized.speed;
					
					auto left   = PULSES_OF(wheels.left, default_wheel_k);
					auto right  = PULSES_OF(wheels.right, default_wheel_k);
					auto rudder = static_cast<short>(PULSES_OF(target.rudder, default_rudder_k));
					
					port << pack_big_endian<ecu<0>::target_speed, int>(left)
					     << pack_big_endian<ecu<1>::target_speed, int>(right)
					     << pack_big_endian<tcu<0>::target_position, short>(rudder);
				}
			});
		
		uint8_t buffer[64];
		while (running) {
			try {
				auto actual = port.read(buffer, sizeof(buffer));
				
				for (auto i = 0; i < actual; ++i) parser(buffer[i]);
				
			} catch (std::exception &) { break; }
		}
		
		running       = false;
		chassis_state = {};
	});
	// endregion
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

chassis::~chassis() {
	if (!running.exchange(false))
		port.break_read();
	
	read_thread.join();
	write_thread.join();
}

autolabor::motor_t<> chassis::left() const {
	return _left;
}

autolabor::motor_t<> chassis::right() const {
	return _right;
}

autolabor::motor_t<> chassis::rudder() const {
	return _rudder;
}

void chassis::enable() {
	try {
		port << pack_big_endian<unit<>::release_stop, uint8_t>(0xff);
	} catch (std::exception &) {}
}

void chassis::disable() {
	try {
		port << autolabor::can::pack<unit<>::emergency_stop>();
	} catch (std::exception &) {}
}

chassis_state_t chassis::state() const {
	return chassis_state;
}

void chassis::set_target(double speed, double rudder) {
	request_time = now();
	target       = {static_cast<float>(speed),
	                static_cast<float>(rudder)};
	legalize_physical(&target, 6 * pi_f);
}

autolabor::odometry_t chassis::odometry() const {
	return _odometry;
}

bool chassis::is_running() const {
	return running;
}
