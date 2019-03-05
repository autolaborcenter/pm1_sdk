//
// Created by ydrml on 2019/2/25.
//

#ifndef PM1_SDK_CHASSIS_H
#define PM1_SDK_CHASSIS_H

#include <mutex>
#include "serial/serial.h"
#include "can/can_define.h"

namespace autolabor {
	namespace pm1 {
		/** 轮速里程计 */
		struct odometry_t { double s, x, y, theta, vx, vy, w; };
		
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
			
			double left() const;
			
			double right() const;
			
			double rudder() const;
			
			void left(double) const;
			
			void right(double) const;
			
			void rudder(double) const;
			
			odometry_t odometry() const;
		
		private:
			/** 串口引用 */
			std::shared_ptr<serial::Serial> port;
			
			/** 电机数据 */
			double _left   = 0,
			       _right  = 0,
			       _rudder = 0;
			
			/** 电机指令 */
			mutable msg_union<int> target_left{0},
			                       target_right{0};
			mutable double         target_rudder;
			
			/** 状态 */
			odometry_t _odometry{};
			
			/** 更新里程计 */
			mutable std::mutex lock;
		};
	}
}


#endif //PM1_SDK_CHASSIS_H
