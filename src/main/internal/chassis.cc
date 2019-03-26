//
// Created by ydrml on 2019/2/25.
//

#include "chassis.hh"

#include <algorithm>
#include <fstream>
#include "serial_extension.h"
#include "can/parser.hh"
#include "can/parse_engine.hh"

extern "C" {
#include "control_model/motor_map.h"
#include "control_model/optimization.h"
}

using namespace autolabor::pm1;

chassis::chassis(const std::string &port_name,
                 const chassis_config_t &chassis_config,
                 float optimize_width,
                 float acceleration)
		: port(new serial::Serial(port_name, 115200,
		                          serial::Timeout(serial::Timeout::max(), 5, 0, 0, 0))),
		  parameters(chassis_config),
		  _odometry({chassis_config}),
		  optimize_width(optimize_width),
		  acceleration(acceleration),
		  mutex0(std::make_shared<std::mutex>()),
		  mutex1(std::make_shared<std::mutex>()),
		  mutex2(std::make_shared<std::mutex>()) {
	using result_t = autolabor::can::parser::result_type;
	
	constexpr static auto odometry_interval = std::chrono::milliseconds(50);
	constexpr static auto rudder_interval   = std::chrono::milliseconds(20);
	constexpr static auto control_timeout   = std::chrono::milliseconds(500);
	constexpr static auto check_timeout     = std::chrono::milliseconds(500);
	
	_left.time = _right.time = _rudder.time = now();
	
	// region check nodes
	{
		*port << autolabor::can::pack<unit<>::state_tx>();
		
		std::string buffer;
		const auto  time = now();
		
		autolabor::can::parse_engine parser(
				[this](const autolabor::can::parser::result &result) {
					if (result.type != result_t::message) return;
					
					auto state = parse_state(*result.message.data.data);
					
					if (unit<ecu<0>>::state_rx::match(result.message))
						_left.state = state;
					else if (unit<ecu<1>>::state_rx::match(result.message))
						_right.state = state;
					else if (unit<tcu<0>>::state_rx::match(result.message))
						_rudder.state = state;
				});
		
		while (port->isOpen()
		       && _left.state != node_state_t::unknown
		       && _right.state != node_state_t::unknown
		       && _rudder.state != node_state_t::unknown) {
			
			*port >> buffer;
			if (!buffer.empty()) parser(*buffer.begin());
			
			if (now() - time > check_timeout)
				throw std::exception("it's not a pm1 chassis");
		}
	}
	// endregion
	
	*port << autolabor::can::pack<ecu<>::timeout>({2, 0}); // 设置超时时间：200 ms
	
	// region ask
	auto port_ptr = port;
	
	// 定时询问前轮
	auto mutex_ptr = mutex0;
	std::thread([port_ptr, mutex_ptr] {
		auto time = now();
		while (port_ptr->isOpen()) {
			{
				std::lock_guard<std::mutex> _(*mutex_ptr);
				if (!port_ptr->isOpen()) break;
				
				time += odometry_interval;
				*port_ptr << autolabor::can::pack<ecu<>::current_position_tx>();
			}
			
			std::this_thread::sleep_until(time);
		}
	}).detach();
	
	// 定时询问后轮
	mutex_ptr = mutex1;
	std::thread([port_ptr, mutex_ptr] {
		auto time = now();
		while (port_ptr->isOpen()) {
			{
				std::lock_guard<std::mutex> _(*mutex_ptr);
				if (!port_ptr->isOpen()) break;
				
				time += rudder_interval;
				*port_ptr << autolabor::can::pack<tcu<0>::current_position_tx>();
			}
			std::this_thread::sleep_until(time);
		}
	}).detach();
	// endregion
	
	// region receive
	mutex_ptr = mutex2;
	std::thread([port_ptr, mutex_ptr, this] {
		std::string buffer;
		
		auto left_ready  = false,
		     right_ready = false;
		auto delta_left  = .0,
		     delta_right = .0;
		auto time        = now();
		auto copy        = this->parameters;
		auto speed       = .0f;
		
		autolabor::can::parse_engine parser(
				[&](const autolabor::can::parser::result &result) {
					if (result.type != result_t::message) return;
					
					auto _now = now();
					
					// 处理
					const auto msg = result.message;
					
					if (ecu<0>::current_position_rx::match(msg)) {
						
						auto value = RAD_OF(get_big_endian<int>(msg), default_wheel_k);
						delta_left = value - _left.position;
						
						_left.update(_now, value);
						
						if (right_ready) {
							if (clear_flag) clear_flag = false;
							else {
								{
									std::lock_guard<std::mutex> _(odometry_protector);
									_odometry += {delta_left * copy.radius,
									              delta_right * copy.radius,
									              _now - time};
								}
								right_ready = false;
								time        = _now;
							}
						} else
							left_ready = true;
						
					} else if (ecu<1>::current_position_rx::match(msg)) {
						
						auto value = RAD_OF(get_big_endian<int>(msg), default_wheel_k);
						delta_right = value - _right.position;
						
						_right.update(_now, value);
						
						if (left_ready) {
							if (clear_flag) clear_flag = false;
							else {
								{
									std::lock_guard<std::mutex> _(odometry_protector);
									_odometry += {delta_left * copy.radius,
									              delta_right * copy.radius,
									              _now - time};
								}
								left_ready = false;
								time       = _now;
							}
						} else
							right_ready = true;
						
					} else if (tcu<0>::current_position_rx::match(msg)) {
						
						auto value = RAD_OF(get_big_endian<short>(msg), default_rudder_k);
						_rudder.update(_now, value);
						
						if (std::isnan(target.rudder) || now() - request_time > control_timeout)
							target         = {0, value};
						
						physical current{speed, value};
						auto     optimized = optimize(&target, &current, this->optimize_width, this->acceleration);
						auto     wheels    = physical_to_wheels(&optimized, &copy);
						speed = optimized.speed;
						
						auto left   = PULSES_OF(wheels.left, default_wheel_k);
						auto right  = PULSES_OF(wheels.right, default_wheel_k);
						auto rudder = static_cast<short>(PULSES_OF(target.rudder, default_rudder_k));
						*port_ptr << pack_big_endian<ecu<0>::target_speed, int>(left)
						          << pack_big_endian<ecu<1>::target_speed, int>(right)
						          << pack_big_endian<tcu<0>::target_position, short>(rudder);
					}
				});
		
		while (port_ptr->isOpen()) {
			std::lock_guard<std::mutex> _(*mutex_ptr);
			if (!port_ptr->isOpen()) break;
			
			*port_ptr >> buffer;
			
			if (!buffer.empty()) parser(*buffer.begin());
		}
	}).detach();
	// endregion
}

chassis::~chassis() {
	std::lock_guard<std::mutex>
			_0(*mutex0),
			_1(*mutex1),
			_2(*mutex2);
	port->close();
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

void chassis::check_state() {
	_left.state   = node_state_t::unknown;
	_right.state  = node_state_t::unknown;
	_rudder.state = node_state_t::unknown;
	*port << autolabor::can::pack<unit<>::state_tx>();
}

void chassis::enable() {
	*port << pack_big_endian<unit<>::release_stop, uint8_t>(0xff);
}

void chassis::disable() {
	*port << autolabor::can::pack<unit<>::emergency_stop>();
}

void chassis::set_target(const physical &t) {
	request_time = now();
	target       = t;
	legalize_physical(&target, 6 * pi_f);
}

odometry_t chassis::odometry() const {
	std::lock_guard<std::mutex> _(odometry_protector);
	return _odometry;
}

void chassis::clear_odometry() {
	std::lock_guard<std::mutex> _(odometry_protector);
	clear_flag = true;
	_odometry  = odometry_t{parameters};
}
