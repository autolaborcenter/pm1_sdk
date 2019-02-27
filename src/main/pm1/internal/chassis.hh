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
		constexpr uint8_t period   = 90;
		constexpr double  pi       = 3.141592653589793238462643383279502884l;
		constexpr double  wheel_k  = 2 * pi / 32000;
		constexpr double  rudder_k = 2 * pi / 16384;
		
		struct motor_info {
			double speed;
			double position;
		};
		
		/** 底盘 */
		class chassis final {
		public:
			/** 绑定特定串口 */
			explicit chassis(const std::string &);
			
			/** 析构 */
			~chassis();
			
			/** 不可复制 */
			chassis(const chassis &) = delete;
			
			/** 不可移动 */
			chassis(chassis &&) = delete;
			
			motor_info left() const;
			
			motor_info right() const;
			
			motor_info rudder() const;
			
			void left(double) const;
			
			void right(double) const;
			
			void rudder(double) const;
		
		private:
			/** 串口引用 */
			std::shared_ptr<serial::Serial> port;
			
			/** 电机数据 */
			motor_info _left{}, _right{}, _rudder{};
		};
	}
}


#endif //PM1_SDK_PM1_H
