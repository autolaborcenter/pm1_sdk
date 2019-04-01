//
// Created by ydrml on 2019/3/25.
//

#ifndef PM1_SDK_EXCEPTION_H
#define PM1_SDK_EXCEPTION_H

namespace autolabor {
	namespace pm1 {
		union union_error_code {
			struct error_code {
				bool no_serial        : 1, // 无可用串口
				     serial_error     : 1, // 串口通信错误
				     not_initialized  : 1, // 未初始化的执行
				     illegal_argument : 1, // 错误参数
				     others           : 1; // 其他
			} bits;
			
			unsigned short code;
		};
	}
}

#endif //PM1_SDK_EXCEPTION_H
