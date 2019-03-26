//
// Created by ydrml on 2019/3/25.
//

#ifndef PM1_SDK_EXCEPTION_H
#define PM1_SDK_EXCEPTION_H

#include "internal/serial/v8stdint.h"

namespace autolabor {
	namespace pm1 {
		union union_error_code {
			struct error_code {
				bool no_serial          : 1, // 无可用串口
				     serial_error       : 1, // 串口通信错误
				     not_initialized : 1, // 未初始化的执行
				     ecu0_offline       : 1, // 子控制器下线
				     ecu1_offline       : 1, // 子控制器下线
				     tcu0_offline       : 1, // 子控制器下线
				     vcu0_offline       : 1, // 子控制器下线
				     mcu0_offline       : 1, // 子控制器下线
				     illegal_argument   : 1, // 错误参数
				     others             : 1; // 其他
			} bits;
			
			uint16_t code;
		};
	}
}

#endif //PM1_SDK_EXCEPTION_H
