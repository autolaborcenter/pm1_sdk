//
// Created by ydrml on 2019/2/27.
//

#ifndef PM1_SDK_PM1_H
#define PM1_SDK_PM1_H

namespace autolabor {
	namespace pm1 {
		namespace mechanical {
			constexpr double pi       = 3.141592653589793238462643383279502884l;
			constexpr double wheel_k  = 2 * pi / 32000;
			constexpr double rudder_k = 2 * pi / 16384;
			constexpr double width    = 0.412;
			constexpr double length   = 0.324;
			constexpr double diameter = 0.211;
			constexpr double radius   = diameter / 2;
		}
	}
}

#endif //PM1_SDK_PM1_H
