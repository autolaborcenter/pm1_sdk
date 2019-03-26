//
// Created by ydrml on 2019/3/25.
//

#ifndef PM1_SDK_EXCEPTION_H
#define PM1_SDK_EXCEPTION_H

#include "internal/serial/v8stdint.h"

namespace autolabor {
	namespace pm1 {
		union union_error_code {
			uint8_t code;
			
			struct error_code {
				bool serial : 1,
				     ecu0 : 1,
				     ecu1 : 1,
				     tcu0 : 1,
				     vcu0 : 1,
				     mcu0 : 1;
			}       bits;
		};
	}
}

#endif //PM1_SDK_EXCEPTION_H
