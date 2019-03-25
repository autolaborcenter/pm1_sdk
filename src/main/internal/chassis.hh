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
		pm1::node_state_t state;
		double            position, speed;
		time_t            time;
		
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
			
			/** 左轮状态 */
			motor_t<> left() const;
			
			/** 右轮状态 */
			motor_t<> right() const;
			
			/** 舵轮状态 */
			motor_t<> rudder() const;
			
			/** 查询状态 */
			void check_state();
			
			/** 锁定 */
			void enable();
			
			/** 解锁 */
			void disable();
			
			void set_target(const physical &);
			
			odometry_t odometry() const;
			
			void clear_odometry();
		
		private:
			/** 串口引用 */
			std::shared_ptr<serial::Serial> port;
			
			/** 电机数据 */
			motor_t<> _left{node_state_t::unknown},
			          _right{node_state_t::unknown},
			          _rudder{node_state_t::unknown};
			
			/** 底盘参数 */
			chassis_config_t parameters;
			
			/** 优化参数 */
			float optimize_width,
			      acceleration;
			
			/** 里程计清零标记 */
			bool clear_flag = true;
			
			/** 里程计 */
			odometry_t _odometry;
			
			mutable std::mutex
					odometry_protector;
			
			std::shared_ptr<std::mutex>
					shared_mutex;
			
			/** 目标运动 */
			physical target{};
			
			/** 最后一次请求的时间 */
			mutable decltype(autolabor::now()) request_time;
		};
	}
}


#endif //PM1_SDK_CHASSIS_H
