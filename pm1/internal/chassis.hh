//
// Created by ydrml on 2019/2/25.
//

#ifndef PM1_SDK_PM1_H
#define PM1_SDK_PM1_H


#include <iostream>
#include "serial/serial.h"
#include "../extensions.h"

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
			
			void test_serial() {
				std::cout << "serial open:"
				          << std::boolalpha << port.isOpen()
				          << std::endl;
				
				std::cout << "Timeout == 1000ms, asking for 1 more byte than written." << std::endl;
				for (int i = 0; i < 10; ++i) {
					const auto test_string = mechdancer::common::join_to_string("", "test", i);
					size_t     bytes_wrote = port.write(test_string);
					
					std::string result = port.read(test_string.length() + 1);
					
					std::cout << "Iteration: " << i
					          << ", Bytes written: " << bytes_wrote
					          << ", Bytes read: " << result.length()
					          << ", String read: " << result
					          << std::endl;
				}
			}
		
		private:
			serial::Serial port;
		};
	}
}


#endif //PM1_SDK_PM1_H
