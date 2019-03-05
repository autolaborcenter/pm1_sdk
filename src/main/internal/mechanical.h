//
// Created by ydrml on 2019/2/27.
//

#ifndef PM1_SDK_PM1_H
#define PM1_SDK_PM1_H


namespace {
	template<class range>
	typename range::t adjust(typename range::t value) {
		static_assert(range::max > range::min, "range must exist");
		constexpr static typename range::t length = range::max - range::min;
		
		while (value < range::min) value += length;
		while (value > range::max) value -= length;
		
		return value;
	}
}

namespace autolabor {
	namespace pm1 {
		namespace mechanical {
			constexpr double pi = 3.141592653589793238462643383279502884l;
			
			constexpr unsigned int encoder_wheel  = 32000;
			constexpr unsigned int encoder_rudder = 16384;
			
			constexpr double wheel_k  = 2 * pi / encoder_wheel;
			constexpr double rudder_k = 2 * pi / encoder_rudder;
			
			constexpr double width    = 0.412;
			constexpr double length   = 0.324;
			constexpr double diameter = 0.211;
			constexpr double radius   = diameter / 2;
			
			constexpr double max_wheel_speed  = 3 * 2 * pi;
			constexpr double max_rudder_speed = pi / 2;
			
			constexpr double max_v = max_wheel_speed * radius;
			constexpr double max_w = 2 * max_wheel_speed * radius / width;
			
			struct state {
				const double rho, theta;
				
				state(double rho, double theta) : rho(rho), theta(theta) {}
			
			private:
				struct half_round {
					using t = double;
					constexpr static t min = -pi,
					                   max = +pi;
				};
				
				const double r     = -length / std::tan(theta),
				             polar = std::atan(max_w / max_v * r);
				const int    sign  = adjust<half_round>(theta) >= 0 ? -1 : +1;
			
			public:
				const double v_rate = sign * rho * std::sin(polar),
				             w_rate = sign * rho * std::cos(polar),
				             v      = max_v * v_rate,
				             w      = max_w * w_rate,
				             left   = v - width / 2 * w,
				             right  = v + width / 2 * w;
				
				/**
				 * 从目标状态构造
				 *
				 * @param v     目标线速度
				 * @param w     目标角速度
				 * @param theta 后轮角度
				 * @return      状态
				 */
				static state from_target(double v, double w) {
					auto theta   = v == 0
					               ? w > 0
					                 ? -mechanical::pi / 2
					                 : w < 0
					                   ? +mechanical::pi / 2
					                   : 0
					               : w == 0
					                 ? 0
					                 : -std::atan(w * mechanical::length / v);
					auto polar   = std::atan(max_w / max_v * -length / std::tan(theta));
					auto sign    = adjust<half_round>(theta) >= 0 ? -1 : +1;
					auto abs_rho = std::hypot(v / max_v, w / max_w);
					return {v >= 0 ? +abs_rho : -abs_rho, theta};
				}
				
				/**
				 * 从两轮轮速构造
				 *
				 * @param left  左轮
				 * @param right 右轮
				 * @param theta 后轮
				 * @return 状态
				 */
				static state from_wheels(double left, double right) {
					return from_target((right + left) / 2,
					                   (right - left) / width);
				}
			};
		}
	}
}


#endif //PM1_SDK_PM1_H
