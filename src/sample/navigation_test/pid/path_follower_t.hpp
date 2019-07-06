//
// Created by ydrml on 2019/7/6.
//

#ifndef PM1_SDK_PATH_FOLLOWER_T_HPP
#define PM1_SDK_PATH_FOLLOWER_T_HPP

#include "virtual_light_sensor_t.hpp"

#include <algorithm>

class path_follower_t {
    using path_iterator_t = typename std::vector<point_t>::iterator;
    
    virtual_light_sensor_t
        sensor;
    path_iterator_t
        local_begin,
        local_end;

public:
    path_follower_t(double senser_x,
                    double sensor_y,
                    double senser_radius
    ) : sensor({senser_x, sensor_y}, senser_radius) {}
    
    void set_path(path_iterator_t new_begin,
                  path_iterator_t new_end) {
        local_begin = new_begin;
        local_end   = new_end;
    }
    
    enum class result_type_t {
        following,
        finish,
        turning,
        failed
    };
    
    struct result_t {
        double speed, rudder;
        
        [[nodiscard]] result_type_t type() const {
            return std::isnan(speed)
                   ? std::isnan(rudder)
                     ? result_type_t::failed
                     : result_type_t::turning
                   : std::isnan(rudder)
                     ? result_type_t::finish
                     : result_type_t::following;
        }
    };
    
    result_t operator()(double x, double y, double theta) {
        using namespace std::chrono_literals;
        
        auto _local_end = local_end;
        auto result     = sensor({x, y}, theta, local_begin, _local_end);
        
        if (local_begin == local_end - 1)
            return {0, NAN};
        
        if (result.tip_begin) {
            auto error = std::atan2((local_begin + 1)->y - local_begin->y,
                                    (local_begin + 1)->x - local_begin->x) - theta;
            while (error > +PI) error -= 2 * PI;
            while (error < -PI) error += 2 * PI;
            
            ++local_begin;
            return {NAN, error};
        }
        
        if (std::isnan(result.error))
            return {NAN, NAN};
        
        return {2.0, -PI / 2 * result.error};
    }
};

#endif //PM1_SDK_PATH_FOLLOWER_T_HPP
