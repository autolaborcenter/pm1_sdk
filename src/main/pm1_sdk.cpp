/** 记录暂停状态 */
volatile bool paused = false;

/** 检查并执行 */
template<class t>
inline result<t> run(const std::function<t()> &code) {
	try {
		return {0, "", code()};
	}
	catch (std::exception &e) {
		return {build_code(e.what()), e.what()};
	}
	catch (...) {
		union_error_code error{};
		error.bits.others = true;
		return {error.code, "unknown and not-exception error"};
	}
}

/** 检查并执行 */
template<>
inline result<void> run(const std::function<void()> &code) {
	try {
		code();
		return {};
	}
	catch (std::exception &e) {
		return {build_code(e.what()), e.what()};
	}
	catch (...) {
		union_error_code error{};
		error.bits.others = true;
		return {error.code, "unknown and not-exception error"};
	}
}

struct process_controller {
	double x0, y0, x1, y1, k;
	
	constexpr process_controller(
			double x0, double y0, double x1, double y1)
			: x0(x0), y0(y0),
			  x1(x1), y1(y1),
			  k((y1 - y0) / (x1 - x0)) {
		if (x0 < 0 || x1 < x0 ||
		    y0 < 0 || y1 < y0)
			throw std::exception("illegal parameters");
	}
	
	inline double operator()(double x) const {
		return x < x0 ? y0
		              : x > x1 ? y1
		                       : k * (x - x0) + y0;
	}
};

const auto max_v = 3 * 2 * pi_f,
           max_w = max_v / default_config.width;

const process_controller
		move_up(0, 0.01, 0.5, max_v),                     // NOLINT(cert-err58-cpp)
		move_down(0.02, 0.01, 10, max_v),                 // NOLINT(cert-err58-cpp)
		rotate_up(0, pi_f / 90, pi_f / 4, max_w),         // NOLINT(cert-err58-cpp)
		rotate_down(pi_f / 900, pi_f / 180, pi_f, max_w); // NOLINT(cert-err58-cpp)

namespace block {
	/** 阻塞等待后轮转动 */
	inline void wait_or_drive(double v, double w) {
		if (paused)
			ptr()->set_target({0, NAN});
		else {
			velocity temp = {static_cast<float>(v), static_cast<float>(w)};
			ptr()->set_target(velocity_to_physical(&temp, &default_config));
		}
	}
	
	/** 阻塞并抑制输出 */
	autolabor::seconds_floating inhibit() {
		return autolabor::measure_time([] {
			while (paused) {
				ptr()->set_target({0, NAN});
				delay(0.02);
			}
		});
	}
	
	/** 按固定控制量运行指定时间 */
	void go_timing(double v, double w, double seconds) {
		auto ending = autolabor::now() + autolabor::seconds_duration(seconds);
		while (autolabor::now() < ending && ptr()->get_state().check_all()) {
			if (paused) ending += inhibit();
			else wait_or_drive(v, w);
			delay(0.02);
		}
	}
}

result<void> autolabor::pm1::go_straight(double speed, double distance) {
	return run<void>([speed, distance] {
		if (speed == 0) {
			if (distance == 0) return;
			throw std::exception(action_cannot_complete);
		}
		if (distance <= 0)
			throw std::exception(illegal_target);
		
		const auto o = ptr()->odometry().s;
		
		while (ptr()->get_state().check_all()) {
			auto current = std::abs(ptr()->odometry().s - o),
			     rest    = distance - current;
			if (rest <= 0) break;
			auto actual = std::min({std::abs(speed),
			                        move_up(current),
			                        move_down(rest)});
			block::wait_or_drive(speed > 0 ? actual : -actual, 0);
			delay(0.02);
		}
		ptr()->set_target({0, NAN});
	});
}

result<void> autolabor::pm1::go_straight_timing(double speed, double time) {
	return run<void>([speed, time] { block::go_timing(speed, 0, time); });
}

result<void> autolabor::pm1::go_arc(double speed, double r, double rad) {
	return run<void>([speed, r, rad] {
		if (r == 0) throw std::exception(illegal_target);
		if (speed == 0) {
			if (rad == 0) return;
			throw std::exception(action_cannot_complete);
		}
		if (rad <= 0)
			throw std::exception(illegal_target);
		
		const auto o = ptr()->odometry().s;
		const auto d = std::abs(r * rad);
		
		while (ptr()->get_state().check_all()) {
			auto current = std::abs(ptr()->odometry().s - o),
			     rest    = d - current;
			if (rest <= 0) break;
			auto actual    = std::min({std::abs(speed),
			                           move_up(current),
			                           move_down(rest)}),
			     available = speed > 0 ? actual : -actual;
			block::wait_or_drive(available, available / r);
			delay(0.02);
		}
		ptr()->set_target({0, NAN});
	});
}

result<void> autolabor::pm1::go_arc_timing(double speed, double r, double time) {
	return run<void>([speed, r, time] {
		if (r == 0) throw std::exception(illegal_target);
		block::go_timing(speed, speed / r, time);
		ptr()->set_target({0, NAN});
	});
}

result<void> autolabor::pm1::turn_around(double speed, double rad) {
	return run<void>([speed, rad] {
		if (speed == 0) {
			if (rad == 0) return;
			throw std::exception(action_cannot_complete);
		}
		if (rad <= 0)
			throw std::exception(illegal_target);
		
		if (rad < 0.01) return;
		auto temp = rad - 0.01;
		
		const auto o = ptr()->odometry().theta;
		while (ptr()->get_state().check_all()) {
			auto current = std::abs(ptr()->odometry().theta - o),
			     rest    = temp - current;
			if (rest <= 0) break;
			auto actual = std::min({std::abs(speed),
			                        rotate_up(current),
			                        rotate_down(rest)});
			block::wait_or_drive(0, speed > 0 ? actual : -actual);
			delay(0.02);
		}
		ptr()->set_target({0, NAN});
	});
}

result<void> autolabor::pm1::turn_around_timing(double speed, double time) {
	return run<void>([speed, time] {
		block::go_timing(0, speed, time);
		ptr()->set_target({0, NAN});
	});
}

result<void> autolabor::pm1::pause() {
	return run<void>([] {
		block::wait_or_drive(0, 0);
		paused = true;
	});
}

result<void> autolabor::pm1::resume() {
	return run<void>([] { paused = false; });
}

void autolabor::pm1::delay(double time) {
	std::this_thread::sleep_for(autolabor::seconds_duration(time));
}

result<void> autolabor::pm1::drive(double v, double w) {
	return run<void>([v, w] {
		velocity temp = {static_cast<float>(v), static_cast<float>(w)};
		ptr()->set_target(velocity_to_physical(&temp, &default_config));
	});
}

result<odometry> autolabor::pm1::get_odometry() {
	return run<odometry>([] {
		auto data = ptr()->odometry() - odometry_mark.load();
		return odometry{data.x,
		                data.y,
		                data.theta,
		                data.vx,
		                data.vy,
		                data.w};
	});
}

result<void> autolabor::pm1::reset_odometry() {
	return run<void>([] { odometry_mark = ptr()->odometry(); });
}

result<void> autolabor::pm1::lock() {
	return run<void>([] { ptr()->disable(); });
}

result<void> autolabor::pm1::unlock() {
	return run<void>([] { ptr()->enable(); });
}

result<chassis_state> autolabor::pm1::get_chassis_state() {
	return run<chassis_state>([] {
		auto temp = ptr()->get_state();
		return chassis_state{(node_state) temp._ecu0,
		                     (node_state) temp._ecu1,
		                     (node_state) temp._tcu};
	});
}
