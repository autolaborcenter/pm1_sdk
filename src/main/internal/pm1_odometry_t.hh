//
// Created by User on 2019/7/25.
//

#ifndef PM1_SDK_PM1_ODOMETRY_T_HH
#define PM1_SDK_PM1_ODOMETRY_T_HH


#include <fstream>

#include <utilities/odometry_t.hpp>
#include <utilities/serial_port/serial_port.hh>
#include <utilities/time/stamped_t.h>
#include "can_define.h"

extern "C" {
#include "control_model/chassis_config_t.h"
}

namespace autolabor {
    namespace pm1 {
        /** 电机信息 */
        struct motor_t { double position, speed; };
        
        /** pm1 里程采集和计算 */
        struct pm1_odometry_t {
            /** 解析结果 */
            enum class result_type : uint8_t { none, left, right };
            
            /** 电机状态缓存 */
            stamped_t<motor_t> _left{}, _right{};
            
            /** 构造器 */
            pm1_odometry_t();
            
            /** 向串口发送里程询问帧 */
            void ask(serial_port &);
            
            /** 解析帧 */
            result_type try_parse(decltype(now()),
                                  const pack_with_data &,
                                  const chassis_config_t &);
            
            /** 获取当前里程计 */
            stamped_t<odometry_t<>> value() const;
        
        private:
            // 日志
            decltype(now()) origin;
            std::ofstream   plot;
            
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
                        decltype(now()),
                        const pack_with_data &,
                        const chassis_config_t &);
        };
    } // namespace pm1
} // namespace autolabor


#endif //PM1_SDK_PM1_ODOMETRY_T_HH
