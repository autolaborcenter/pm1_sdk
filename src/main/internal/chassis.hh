//
// Created by ydrml on 2019/2/25.
//

#ifndef PM1_SDK_CHASSIS_H
#define PM1_SDK_CHASSIS_H

#include <atomic>
#include <thread>
#include <vector>
#include <shared_mutex>

#include "serial/serial_port.hh"
#include "can_define.h"
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
            /** 默认参数 */
            const static float
                default_optimize_width,
                default_acceleration,
                default_max_v,
                default_max_w,
                default_max_wheel_speed;
            
            /** 机械参数 */
            chassis_config_t config;
            
            /** 控制参数 */
            float optimize_width,
                  acceleration,
                  max_v,
                  max_w,
                  max_wheel_speed;
            
            explicit chassis(const std::string &port_name);
            
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
            
            /** 检查状态 */
            chassis_state_t state() const;
            
            /** 目标状态 */
            node_state_t target_state() const;
            
            /** 读取里程计 */
            odometry_t odometry() const;
            
            /** 线程是否正常运行 */
            bool is_threads_running() const;
            
            /** 设置使能目标 */
            void set_enabled_target(bool);
            
            /** 设置目标控制量 */
            void set_target(double speed, double rudder);
            
            /** 重设舵轮零位 */
            void reset_rudder();
        
        private:
            /** 串口引用 */
            serial_port port;
            
            /** 节点状态 */
            chassis_state_t chassis_state{};
            
            /** 电机数据 */
            motor_t<> _left{},
                      _right{},
                      _rudder{};
            
            /** 里程计 */
            std::atomic<odometry_t> _odometry{};
            
            /** 底层线程是否运行 */
            volatile bool running;
            
            std::thread read_thread,
                        write_thread;
            
            /** 托管动作锁 */
            std::shared_mutex action_mutex;
            
            /** 目标设定锁 */
            std::mutex target_mutex;
            
            /** 目标运动 */
            physical target{};
            
            /** 使能目标状态 */
            bool enabled_target;
            
            /** 最后一次请求的时间 */
            decltype(autolabor::now()) request_time;
        };
    }
}


#endif //PM1_SDK_CHASSIS_H
