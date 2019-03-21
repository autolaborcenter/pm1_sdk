//
// Created by ydrml on 2019/2/25.
//

#ifndef PM1_SDK_CHASSIS_H
#define PM1_SDK_CHASSIS_H

#include <mutex>
#include "serial/serial.h"
#include "can_define.h"
#include "odometry_t.hh"
#include "time_extensions.h"

extern "C" {
#include "control_model/model.h"
#include "control_model/optimization.h"
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
			/**
			 *
			 * @param port         串口名字
			 * @param acceleration 加速度（比例/秒）
			 * @param chassis_config       底盘配置
			 */
			explicit chassis(const std::string &port_name,
			                 const chassis_config_t & = default_config,
			                 float optimize_width = pi_f / 4,
			                 float acceleration = INFINITY);
			
			/** 析构 */
			~chassis();
			
			/** 不可复制 */
			chassis(const chassis &) = delete;
			
			/** 不可移动 */
			chassis(chassis &&) = delete;
			
			motor_t<> left() const;
			
			motor_t<> right() const;
			
			motor_t<> rudder() const;
			
			void set_target(const physical &) const;
			
			odometry_t odometry() const;
			
			void clear_odometry();
		
		private:
			/** 串口引用 */
			std::shared_ptr<serial::Serial> port;
			
			/** 电机数据 */
			motor_t<> _left{},
			          _right{},
			          _rudder{};
			
			/** 底盘参数 */
			chassis_config_t parameters;
			
			/** 里程计清零标记 */
			bool clear_flag = true;
			
			/** 里程计 */
			odometry_t _odometry;
			
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
