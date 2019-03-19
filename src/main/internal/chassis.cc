//
// Created by ydrml on 2019/2/25.
//

#include "chassis.hh"

#include <algorithm>
#include "serial_extension.h"
#include "can/parser.hh"
#include "can/parse_engine.hh"

using namespace autolabor::pm1;

namespace mechanical {
	constexpr int encoder_wheel  = +32000;
	constexpr int encoder_rudder = -16384;
	
	constexpr double wheel_k  = 2 * PI_F / encoder_wheel;
	constexpr double rudder_k = 2 * PI_F / encoder_rudder;
}

/** 控制优化参数 */
constexpr double optimize_limit = PI_F / 3;

/** 连续控制优化 */
inline physical optimize(const physical &target, double rudder);

chassis::chassis(const std::string &port_name, const chassis_config_t &parameters)
		: port(new serial::Serial(port_name, 115200,
		                          serial::Timeout(serial::Timeout::max(), 5, 0, 0, 0))),
		  parameters(parameters),
		  _odometry({parameters}) {
	using result_t = autolabor::can::parser::result_type;
	
	constexpr static auto odometry_interval = std::chrono::milliseconds(50);
	constexpr static auto rudder_interval   = std::chrono::milliseconds(20);
	constexpr static auto control_timeout   = std::chrono::milliseconds(200);
	
	_left.time = _right.time = _rudder.time = now();
	
	// region check nodes
	{
		*port << autolabor::can::pack<unit<>::state_tx>();
		
		std::string buffer;
		const auto  time = now();
		auto        ecu0 = false,
		            ecu1 = false,
		            tcu0 = false;
		
		autolabor::can::parse_engine parser(
				[&ecu0, &ecu1, &tcu0](const autolabor::can::parser::result &result) {
					if (result.type != result_t::message) return;
					
					const auto msg = result.message;
					
					if (unit<ecu<0>>::state_rx::match(msg))
						ecu0 = *msg.data.data;
					else if (unit<ecu<1>>::state_rx::match(msg))
						ecu1 = *msg.data.data;
					else if (unit<tcu<0>>::state_rx::match(msg))
						tcu0 = *msg.data.data;
				});
		
		while (port->isOpen()
		       && !(ecu0 && ecu1 && tcu0)
		       && now() - time < seconds_duration(1)) {
			
			try { buffer = port->read(); }
			catch (std::exception &) { buffer = ""; }
			
			if (!buffer.empty()) parser(*buffer.begin());
		}
		
		if (!ecu0 || !ecu1 || !tcu0)
			throw std::exception("it's not a pm1 chassis");
	}
	// endregion
	
	*port << autolabor::can::pack<ecu<>::timeout>({2, 0}); // 设置超时时间：200 ms
	
	// region ask
	auto port_ptr = port;
	
	// 定时询问前轮
	std::thread([port_ptr] {
		auto time = now();
		while (port_ptr->isOpen()) {
			time += odometry_interval;
			try {
				*port_ptr << autolabor::can::pack<ecu<>::current_position_tx>();
			} catch (std::exception &) {}
			std::this_thread::sleep_until(time);
		}
	}).detach();
	
	// 定时询问后轮
	std::thread([port_ptr] {
		auto time = now();
		while (port_ptr->isOpen()) {
			time += rudder_interval;
			try {
				*port_ptr << autolabor::can::pack<tcu<0>::current_position_tx>();
			} catch (std::exception &) {}
			std::this_thread::sleep_until(time);
		}
	}).detach();
	// endregion
	
	// region receive
	std::thread([port_ptr, this] {
		std::string buffer;
		
		auto left_ready  = false,
		     right_ready = false;
		auto delta_left  = .0,
		     delta_right = .0;
		auto time        = now();
		auto copy        = this->parameters;
		
		autolabor::can::parse_engine parser(
				[&](const autolabor::can::parser::result &result) {
					if (result.type != result_t::message) return;
					
					auto _now = now();
					
					// 处理
					const auto msg = result.message;
					
					if (ecu<0>::current_position_rx::match(msg)) {
						
						auto value = get_big_endian<int>(msg) * mechanical::wheel_k;
						delta_left = value - _left.position;
						
						_left.update(_now, value);
						
						if (right_ready) {
							if (clear_flag) clear_flag = false;
							else {
								{
									std::lock_guard<std::mutex> _(lock);
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
						
						auto value = get_big_endian<int>(msg) * mechanical::wheel_k;
						delta_right = value - _right.position;
						
						_right.update(_now, value);
						
						if (left_ready) {
							if (clear_flag) clear_flag = false;
							else {
								{
									std::lock_guard<std::mutex> _(lock);
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
						
						auto value = get_big_endian<short>(msg) * mechanical::rudder_k;
						_rudder.update(_now, value);
						
						int   left, right;
						short rudder;
						
						if (!std::isnan(target.rudder) && now() - request_time < control_timeout) {
							// 200 ms 内，参数有效
							auto optimized = optimize(target, _rudder.position);
							auto wheels    = physical_to_wheels(&optimized, &copy);
							left   = static_cast<int>(wheels.left / copy.radius / mechanical::wheel_k);
							right  = static_cast<int>(wheels.right / copy.radius / mechanical::wheel_k);
							rudder = static_cast<short>(target.rudder / mechanical::rudder_k);
							
						} else {
							// 停机
							left   = 0;
							right  = 0;
							rudder = static_cast<short>(value / mechanical::rudder_k);
						}
						
						*port_ptr << pack_big_endian<ecu<0>::target_speed, int>(left)
						          << pack_big_endian<ecu<1>::target_speed, int>(right)
						          << pack_big_endian<tcu<0>::target_position, short>(rudder);
					}
				});
		
		while (port_ptr->isOpen()) {
			try { buffer = port_ptr->read(); }
			catch (std::exception &) { buffer = ""; }
			
			if (!buffer.empty()) parser(*buffer.begin());
		}
	}).detach();
	// endregion
}

chassis::~chassis() {
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

odometry_t chassis::odometry() const {
	std::lock_guard<std::mutex> _(lock);
	return _odometry;
}

void chassis::set_target(const physical &t) const {
	request_time = now();
	target       = t;
}

void chassis::clear_odometry() {
	std::lock_guard<std::mutex> _(lock);
	clear_flag = true;
	_odometry  = odometry_t{parameters};
}

inline physical optimize(const physical &target, double rudder) {
	auto difference = std::abs(target.rudder - rudder);
	return {static_cast<float>(difference > optimize_limit ? 0 : (1 - difference / optimize_limit) * target.speed),
	        static_cast<float>(rudder)};
}
