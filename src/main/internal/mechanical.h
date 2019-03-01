//
// Created by ydrml on 2019/2/27.
//

#ifndef PM1_SDK_PM1_H
#define PM1_SDK_PM1_H

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
			
			constexpr double max_wheel_speed = 3 * 2 * pi;
			
			constexpr double max_v = max_wheel_speed * radius;
			constexpr double max_w = 2 * max_wheel_speed * radius / width;
		}
	}
}

#endif //PM1_SDK_PM1_H
