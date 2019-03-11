//
// Created by ydrml on 2019/2/25.
//

#include "chassis.hh"

#include <algorithm>
#include "can/parser.hh"
#include "can/parse_engine.hh"

using namespace autolabor::pm1;

/** 发送数据包 */
template<class t>
inline const std::shared_ptr<serial::Serial> &operator<<(
		const std::shared_ptr<serial::Serial> &,
		const autolabor::can::msg_union<t> &);

/** 获取存储区中大端存储的第一个数据 */
template<class t>
inline t get_first(const uint8_t *);

/** 数据大端打包到数据域 */
template<class pack_info_t, class data_t>
inline auto pack_into(const autolabor::can::msg_union<data_t> &value)
-> decltype(autolabor::can::pack<pack_info_t>());

/** 里程计更新信息 */
template<class time_unit = std::chrono::duration<double, std::ratio<1>>>
struct odometry_update_info { double d_left, d_rigth; time_unit d_t; };

/** 更新轮速里程计 */
inline void operator+=(odometry_t &, odometry_update_info<>);

/** 控制优化参数 */
constexpr double optimize_limit = mechanical::pi / 3;

/** 连续控制优化 */
inline mechanical::state optimize(const mechanical::state &target, double rudder);

chassis::chassis(const std::string &port_name)
		: port(new serial::Serial(port_name, 115200,
		                          serial::Timeout(serial::Timeout::max(), 5, 0, 0, 0))) {
	using result_t = autolabor::can::parser::result_type;
	
	_left.time = _right.time = _rudder.time = now();
	
	// region check nodes
	{
		port << autolabor::can::pack<unit<>::state_tx>();
		
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
		
		//		if (!ecu0 || !ecu1 || !tcu0)
		//			throw std::exception("it's not a pm1 chassis");
	}
	// endregion
	
	port << autolabor::can::pack<ecu<>::timeout>({2, 0}); // 设置超时时间：200 ms
	
	// region ask
	auto port_ptr = port;
	
	// 定时询问前轮
	std::thread([port_ptr] {
		auto time = now();
		while (port_ptr->isOpen()) {
			time += std::chrono::milliseconds(100);
			try {
				port_ptr << autolabor::can::pack<ecu<>::current_position_tx>();
			} catch (std::exception &) {}
			std::this_thread::sleep_until(time);
		}
	}).detach();
	
	// 定时询问后轮
	std::thread([port_ptr] {
		auto time = now();
		while (port_ptr->isOpen()) {
			time += std::chrono::milliseconds(100);
			try {
				port_ptr << autolabor::can::pack<tcu<0>::current_position_tx>();
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
		
		autolabor::can::parse_engine parser(
				[&](const autolabor::can::parser::result &result) {
					if (result.type != result_t::message) return;
					
					auto _now = now();
					
					// 处理
					const auto msg   = result.message;
					const auto bytes = msg.data.data;
					
					if (ecu<0>::current_position_rx::match(msg)) {
						
						auto value = get_first<int>(bytes) * mechanical::wheel_k;
						delta_left = (value - _left.position) * mechanical::radius;
						
						_left.speed    = delta_left / (_now - _left.time).count();
						_left.position = value;
						_left.time     = _now;
						
						if (right_ready) {
							if (clear_flag) clear_flag = false;
							else {
								std::lock_guard<std::mutex> _(lock);
								right_ready = false;
								
								_odometry += {delta_left, delta_right, _now - time};
								time        = _now;
							}
						} else
							left_ready = true;
						
					} else if (ecu<1>::current_position_rx::match(msg)) {
						
						auto value = get_first<int>(bytes) * mechanical::wheel_k;
						delta_right = (value - _right.position) * mechanical::radius;
						
						_right.speed    = delta_right / (_now - _right.time).count();
						_right.position = value;
						_right.time     = _now;
						
						if (left_ready) {
							if (clear_flag) clear_flag = false;
							else {
								std::lock_guard<std::mutex> _(lock);
								left_ready = false;
								
								_odometry += {delta_left, delta_right, _now - time};
								time       = _now;
							}
						} else
							right_ready = true;
						
					} else if (tcu<0>::current_position_rx::match(msg)) {
						
						_rudder.position = get_first<short>(bytes) * mechanical::rudder_k;
						
						autolabor::can::msg_union<int>   left{}, right{};
						autolabor::can::msg_union<short> temp{};
						
						if (now() - request_time < std::chrono::milliseconds(200)) {
							auto optimized = optimize(*target, _rudder.position);
							left.data  = static_cast<int> (optimized.left / mechanical::radius / mechanical::wheel_k);
							right.data = static_cast<int> (optimized.right / mechanical::radius / mechanical::wheel_k);
							temp.data  = static_cast<short> (target->rudder / mechanical::rudder_k);
						} else {
							left.data =
							right.data = 0;
							temp.data = static_cast<short> (_rudder.position / mechanical::rudder_k);
						}
						
						port_ptr << pack_into<ecu<0>::target_speed, int>(left)
						         << pack_into<ecu<1>::target_speed, int>(right)
						         << pack_into<tcu<0>::target_position, short>(temp);
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

void chassis::set_state(double rho, double rudder) const {
	request_time = now();
	target       = mechanical::state::make_shared(rho, rudder);
}

void chassis::set_target(double v, double w) const {
	request_time = now();
	target       = mechanical::state::from_target(v, w);
}

void chassis::clear_odometry() {
	clear_flag = true;
	_odometry  = {};
}

template<class t>
inline const std::shared_ptr<serial::Serial> &operator<<(
		const std::shared_ptr<serial::Serial> &port,
		const autolabor::can::msg_union<t> &msg) {
	try { port->write(msg.bytes, sizeof(t)); } catch (...) {}
	return port;
}

template<class t>
inline t get_first(const uint8_t *bytes) {
	autolabor::can::msg_union<t> temp{};
	std::reverse_copy(bytes, bytes + sizeof(t), temp.bytes);
	return temp.data;
}

template<class pack_info_t, class data_t>
inline auto
pack_into(const autolabor::can::msg_union<data_t> &value)
-> decltype(autolabor::can::pack<pack_info_t>()) {
	std::array<uint8_t, 8> buffer{};
	std::reverse_copy(value.bytes, value.bytes + sizeof(data_t), buffer.data());
	return autolabor::can::pack<pack_info_t>(buffer);
}

/**
 * 计算机器人坐标系下的里程计
 *
 * @param delta_left  左轮变化量
 * @param delta_right 右轮变化量
 * @param theta       车身转角
 * @param x           横坐标相对变化
 * @param y           纵坐标相对变化
 */
inline void calculate_odometry(
		double delta_left,
		double delta_right,
		double &theta,
		double &x,
		double &y) {
	theta = (delta_right - delta_left) / mechanical::width;
	if (theta == 0) {
		x = delta_left;
		y = 0;
	} else {
		const auto sin = std::sin(theta / 2);
		const auto cos = std::cos(theta / 2);
		const auto r   = (delta_left + delta_right) / 2 / theta;
		const auto d   = 2 * r * sin;
		x = d * cos;
		y = d * sin;
	}
}

/**
 * 坐标系旋转
 *
 * @param x     横坐标
 * @param y     纵坐标
 * @param theta 旋转弧度
 */
inline void rotate(double &x,
                   double &y,
                   double theta) {
	double _;
	auto   sin = std::sin(theta);
	auto   cos = std::cos(theta);
	_ = x * cos - y * sin;
	y = x * sin + y * cos;
	x = _;
}

inline void operator+=(odometry_t &odometry,
                       odometry_update_info<> info) {
	odometry.s += (info.d_left + info.d_rigth) / 2;
	
	double theta, x, y;
	calculate_odometry(info.d_left, info.d_rigth, theta, x, y);
	rotate(x, y, odometry.theta);
	
	odometry.x += x;
	odometry.y += y;
	odometry.theta += theta;
	
	odometry.vx = x / info.d_t.count();
	odometry.vy = y / info.d_t.count();
	odometry.w  = theta / info.d_t.count();
}

inline mechanical::state optimize(const mechanical::state &target, double rudder) {
	const auto difference = std::abs(target.rudder - rudder);
	return {difference > optimize_limit ? 0 : (1 - difference / optimize_limit) * target.rho, rudder};
}
