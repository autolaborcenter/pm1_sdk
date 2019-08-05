//
// Created by User on 2019/7/25.
//

#ifndef PM1_SDK_ODOMETRY_H
#define PM1_SDK_ODOMETRY_H


#include <utilities/odometry_t.hpp>
#include <utilities/serial_port/serial_port.hh>
#include <utilities/time/stamped_t.h>
#include "control_model/chassis_config_t.h"
#include "can_define.h"

namespace autolabor {
    namespace pm1 {
        /**
         * 推算里程增量
         * @param left   左轮转角增量
         * @param right  右轮转角增量
         * @param config 底盘结构参数
         * @return 里程增量
         */
        odometry_t<odometry_type::delta> wheels_to_odometry(
            double left,
            double right,
            const chassis_config_t &config);
    
        /** 电机信息 */
        struct motor_t { double position, speed; };
    
        /** pm1 里程采集和计算 */
        struct pm1_odometry_t {
            /** 解析结果 */
            enum class result_type : uint8_t { none, left, right };
        
            /** 电机状态缓存 */
            stamped_t<motor_t> _left{}, _right{};
        
            /** 向串口发送里程询问帧 */
            void ask(serial_port &port);
        
            /** 解析帧 */
            result_type try_parse(decltype(now()) _now,
                                  const pack_with_data &msg,
                                  const chassis_config_t &config);
        
            /** 获取当前里程计 */
            stamped_t<odometry_t<>> value() const;
    
        private:
            // 电机通信应答序号
            std::atomic_ulong
                wheels_seq;
        
            // 电机更新信息记录
            struct { unsigned long seq; double last; }
                l_mark{},
                r_mark{};
        
            // 里程计更新锁
            mutable std::mutex
                update_lock;
        
            // 里程计缓存
            stamped_t<odometry_t<>>
                _odometry{};
        
            // 进行更新
            void update(bool left,
                        decltype(now()) _now,
                        const pack_with_data &msg,
                        const chassis_config_t &config);
        };
        
    } // namespace pm1
} // namespace autolabor


#endif //PM1_SDK_ODOMETRY_H
