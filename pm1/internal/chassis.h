//
// Created by ydrml on 2019/2/25.
//

#ifndef PM1_SDK_PM1_H
#define PM1_SDK_PM1_H


#include "serial/serial.h"

namespace autolabor {
	namespace pm1 {
		
		/**
		 * 底盘
		 */
		class chassis {
		public:
			/** 构造时获取资源 */
			explicit chassis(const std::string &port)
					: port(port, 9600, serial::Timeout::simpleTimeout(1000)) {}
			
			/** 析构时释放资源 */
			~chassis();
			
			/** 不可复制 */
			chassis(const chassis &others) = delete;
			
			/** 不可移动 */
			chassis(chassis &&others) = delete;
		
		private:
			serial::Serial port;
		};
	}
}


#endif //PM1_SDK_PM1_H
