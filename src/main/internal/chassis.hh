//
// Created by ydrml on 2019/2/25.
//

#ifndef PM1_SDK_CHASSIS_H
#define PM1_SDK_CHASSIS_H

#include <mutex>
#include "serial/serial.h"
#include "can_define.h"
#include "mechanical.h"
#include "odometry_t.hh"
#include "time_extensions.h"

extern "C" {
#include "control_model/model.h"
}

namespace autolabor {
	/** 电机信息 */
	template<class time_t = decltype(now())>
	struct motor_t {
		double position, speed;
		time_t time;
		
		void update(time_t _now, double value) {
			auto delta = value - position;
			
			autolabor::seconds_floating dt = _now - time;
			speed    = delta / dt.count();
			position = value;
			time     = _now;
		}
	};
	
	namespace pm1 {
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
			
			motor_t<> left() const;
			
			motor_t<> right() const;
			
			motor_t<> rudder() const;
			
			void set_state(double, double) const;
			
			void set_target(double, double) const;
			
			odometry_t odometry() const;
			
			void clear_odometry();
		
		private:
			/** 串口引用 */
			std::shared_ptr<serial::Serial> port;
			
			/** 电机数据 */
			motor_t<> _left{},
			          _right{},
			          _rudder{};
			
			/** 里程计清零标记 */
			bool clear_flag = true;
			
			/** 里程计 */
			odometry_t _odometry{};
			
			/** 里程计更新锁 */
			mutable std::mutex lock;
			
			/** 目标运动 */
			mutable physical target{};
			
			/** 最后一次请求的时间 */
			mutable decltype(autolabor::now()) request_time;
		};
	}
}


#endif //PM1_SDK_CHASSIS_H
