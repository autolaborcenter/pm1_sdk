//
// Created by ydrml on 2019/7/6.
//

#ifndef PM1_SDK_PATH_FOLLOWER_T_HPP
#define PM1_SDK_PATH_FOLLOWER_T_HPP


#include "virtual_light_sensor_t.hpp"

#include <algorithm>
#include <internal/control_model/pi.h>

namespace path_follower {
    /**
     * 循线控制器
     */
    template<class container_t>
    class path_follower_t {
        static_assert(std::is_same<typename container_t::value_type, point_t>::value,
                      "path_follower must be a container of points");
        
        // 路径迭代器
        using path_iterator_t = typename container_t::iterator;
        
        virtual_light_sensor_t
            sensor;
        path_iterator_t
            local_begin,
            local_end;
    
        double d = 0, i = 0;
    
    public:
        double speed = 0.1;
        
        /**
         * 构造器
         * @param senser_x      传感器相对位置 x
         * @param sensor_y      传感器相对位置 y
         * @param senser_radius 感应区域半径
         */
        path_follower_t(double senser_x,
                        double sensor_y,
                        double senser_radius)
            : sensor({senser_x, sensor_y}, senser_radius) {}
        
        /**
         * 输入路径（依赖外部存储）
         * @param new_begin 路径起点迭代器
         * @param new_end   路径终点迭代器
         */
        void set_path(path_iterator_t new_begin,
                      path_iterator_t new_end) {
            local_begin = new_begin;
            local_end   = new_end;
        }
        
        /** 循线状态 */
        enum class following_state_t {
            following,
            finish,
            turning,
            failed
        };
        
        /** 计算结果状态界定 */
        struct result_t {
            double speed, rudder;
            
            [[nodiscard]] following_state_t type() const {
                return std::isnan(speed)
                       ? std::isnan(rudder)
                         ? following_state_t::failed
                         : following_state_t::turning
                       : std::isnan(rudder)
                         ? following_state_t::finish
                         : following_state_t::following;
            }
        };
        
        /**
         * 运行循线计算
         * @param x     机器人位置 x
         * @param y     机器人位置 y
         * @param theta 机器人方向
         */
        result_t operator()(double x, double y, double theta, double rudder) {
            auto end_ignore = local_end;
            auto result     = sensor({x, y}, theta, local_begin, end_ignore);
            
            // 路径丢失
            if (result.local_count == 0)
                return {NAN, NAN};
            // 正常情况
            if (result.tip_order == 255) {
                i += result.error;
                if (std::abs(result.error) < 0.01 || result.error * d < 0)
                    i = 0;
        
                auto dd = result.error - d;
                d = result.error;
        
                std::cout << rudder << ' '
                          << result.error << ' '
                          << 5 * dd << ' '
                          << 0.02 * i << std::endl;
                return {std::max(.0, speed - std::abs(result.error) / 30),
                        std::clamp(M_PI / 12 * (result.error + 5 * dd + 0.02 * i),
                                   -M_PI / 2.0,
                                   +M_PI / 2.0)};
            }
            
            
            auto direction = local_begin + result.tip_order;
            // 到达路径终点
            if (direction == local_end)
                return {0, NAN};
            // 遭遇尖点
            auto error = std::atan2(direction->y - local_begin->y,
                                    direction->x - local_begin->x) - theta;
            while (error > +M_PI) error -= 2 * M_PI;
            while (error < -M_PI) error += 2 * M_PI;
            ++local_begin;
            return {NAN, error};
        }
    };
}


#endif //PM1_SDK_PATH_FOLLOWER_T_HPP
