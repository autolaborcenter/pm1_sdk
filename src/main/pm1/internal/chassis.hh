//
// Created by ydrml on 2019/2/25.
//

#ifndef PM1_SDK_PM1_H
#define PM1_SDK_PM1_H


#include <iostream>
#include <thread>
#include "serial/serial.h"
#include "../extensions.h"

namespace autolabor {
	namespace pm1 {
		
		/**
		 * 底盘
		 */
		class chassis final {
		public:
			/** 绑定特定串口 */
			explicit chassis(const std::string &port_name);
			
			/** 析构 */
			~chassis();
			
			/** 不可复制 */
			chassis(const chassis &others) = delete;
			
			/** 不可移动 */
			chassis(chassis &&others) = delete;
			
			/** 测试串口 */
			void test_serial();
			
			/** 获取编码器值 */
			// std::pair<double, double> getRecorder();
		
		private:
			/**
			 * 串口引用
			 */
			std::shared_ptr<serial::Serial> port;
			
			/**
			 * 读取一个字节
			 * @param byte 读出的字节
			 * @return 是否成功读取
			 */
			bool readByte(unsigned char &byte);
		};
	}
}


#endif //PM1_SDK_PM1_H
