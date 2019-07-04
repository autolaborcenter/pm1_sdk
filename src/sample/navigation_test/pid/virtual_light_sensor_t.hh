//
// Created by User on 2019/7/4.
//

#ifndef PM1_SDK_VIRTUAL_LIGHT_SENSOR_T_HH
#define PM1_SDK_VIRTUAL_LIGHT_SENSOR_T_HH


#include "point_t.h"
#include "shape_t.hpp"

/**
 * 虚拟光电传感器
 */
struct virtual_light_sensor_t {
    virtual_light_sensor_t();
    
    void set_pose(point_t position, double direction);
    
    template<class t>
    [[nodiscard]] double
    operator()(t &local_begin, t &local_end) const {
        take_once(local_begin,
                  local_end,
                  [&](point_t point) { return range.check_inside(point); });
        size_t local_size = local_end - local_begin;
        if (local_size < 2) return NAN;
        
        auto shape = range.to_vector();
        
        auto index0 = max_by(shape,
                             [target = *(local_end - 1)](point_t point) {
                                 auto dx = target.x - point.x,
                                      dy = target.y - point.y;
                                 return -dx * dx - dy * dy;
                             }),
             index1 = max_by(shape,
                             [target = *local_begin](point_t point) {
                                 auto dx = target.x - point.x,
                                      dy = target.y - point.y;
                                 return -dx * dx - dy * dy;
                             });
        
        if (index1 < index0) index1 += range.point_count();
        
        shape.resize(local_size + index1 - index0);
        std::copy(local_begin, local_end, shape.begin());
        for (auto i = local_size; i < shape.size(); ++i)
            shape[i] = range[(index0++) % range.point_count()];
        
        return 2 * (0.5 - any_shape(shape).size() / range.size());
    }

private:
    circle_t range;
};


#endif //PM1_SDK_VIRTUAL_LIGHT_SENSOR_T_HH
