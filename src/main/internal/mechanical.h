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
			
			constexpr double wheel_k  = +2 * pi / encoder_wheel;
			constexpr double rudder_k = -2 * pi / encoder_rudder;
			
			constexpr double width    = 0.4205;
			constexpr double length   = 0.3240;
			constexpr double diameter = 0.2074;
			constexpr double radius   = diameter / 2;
			
			constexpr double max_wheel_speed  = 3 * 2 * pi;
			constexpr double max_rudder_speed = pi / 2;
			
			constexpr double max_v = max_wheel_speed * radius;
			constexpr double max_w = 2 * max_wheel_speed * radius / width;
			
			struct state {
				const double rho, rudder;
				
				/**
				 * 构造器
				 *
				 * @param rho    equals to sqrt(v^2 + w^2), but with sign
				 * @param rudder if rudder is 0, it should be `+0`, rather than `-0`
				 */
				state(double rho, double rudder) :
						rho(rho), rudder(rudder == 0 ? +0 : rudder) {}
				
				const int    sign   = adjust<half_round>(rudder) >= 0 ? -1 : +1;
				const double r      = -length / std::tan(rudder),   // 转弯半径
				             polar  = std::atan(max_w / max_v * r), // 速度向量极角
				             v_rate = sign * rho * std::sin(polar), // 线速度比率
				             w_rate = sign * rho * std::cos(polar), // 角速度比率
				             v      = max_v * v_rate,               // 线速度
				             w      = max_w * w_rate,               // 角速度
				             left   = v - width / 2 * w,            // 左轮线速度
				             right  = v + width / 2 * w;            // 右轮线速度
				
				/** 构造智能指针 */
				inline static std::shared_ptr<state>
				make_shared(double rho, double rudder) {
					return std::make_shared<state>(rho, rudder);
				}
				
				/**
				 * 从目标状态构造
				 *
				 * @param v     目标线速度
				 * @param w     目标角速度
				 * @param theta 后轮角度
				 * @return      状态
				 */
				inline static std::shared_ptr<state>
				from_target(double v, double w) {
					auto rudder  = v == 0
					               ? w > 0
					                 ? -mechanical::pi / 2
					                 : +mechanical::pi / 2
					               : -std::atan(w * mechanical::length / v);
					auto abs_rho = std::hypot(v / max_v, w / max_w);
					return make_shared(v >= 0 ? +abs_rho : -abs_rho, rudder);
				}
			
			private:
				struct half_round {
					using t = double;
					constexpr static t min = -pi,
					                   max = +pi;
				};
			};
		}
	}
}


#endif //PM1_SDK_PM1_H
