//
// Created by ydrml on 2019/2/25.
//

#ifndef PM1_SDK_CHASSIS_H
#define PM1_SDK_CHASSIS_H

#include <iostream>
#include <thread>
#include "serial/serial.h"
#include "can/can_define.h"

namespace autolabor {
	namespace pm1 {
		constexpr uint8_t period = 90;
		
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
			
			/** 电机指令 */
			mutable msg_union<int>   target_left, target_right;
			mutable msg_union<short> target_rudder;
		};
	}
}


#endif //PM1_SDK_CHASSIS_H
