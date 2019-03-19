//
// Created by ydrml on 2019/2/27.
//

#ifndef PM1_SDK_PM1_H
#define PM1_SDK_PM1_H


namespace autolabor {
	namespace pm1 {
		namespace mechanical {
			constexpr double pi = 3.141592653589793238462643383279502884l;
			
			constexpr int encoder_wheel  = +32000;
			constexpr int encoder_rudder = -16384;
			
			constexpr double wheel_k  = 2 * pi / encoder_wheel;
			constexpr double rudder_k = 2 * pi / encoder_rudder;
		}
	}
}


#endif //PM1_SDK_PM1_H
