//
// Created by User on 2019/7/4.
//

#ifndef PM1_SDK_VIRTUAL_LIGHT_SENSOR_T_HPP
#define PM1_SDK_VIRTUAL_LIGHT_SENSOR_T_HPP


#include "point_t.h"
#include "shape_t.hpp"

/**
 * 虚拟光电传感器
 */
struct virtual_light_sensor_t {
    virtual_light_sensor_t(
        point_t position,
        double radius
    ) : _position(position),
        range(32, radius) {}
    
    struct result_t {
        size_t local_count;
        double local_size,
               local_struct;
        double error;
    };
    
    template<class t>
    [[nodiscard]] result_t
    operator()(point_t position,
               double direction,
               point_t &ppppppp,
               t &local_begin,
               t &local_end);

private:
    point_t  _position;
    circle_t range;
};

template<class t>
virtual_light_sensor_t::result_t
virtual_light_sensor_t::operator()(
    point_t position,
    double direction,
    point_t &ppppppp,
    t &local_begin,
    t &local_end
) {
    auto   cos = std::cos(direction),
           sin = std::sin(direction);
    
    range.center       = ppppppp = {
        position.x + cos * _position.x - sin * _position.y,
        position.y + sin * _position.x + cos * _position.y,
    };
    range.direction    = direction;
    
    take_once(local_begin,
              local_end,
              [&](point_t point) { return range.check_inside(point); });
    size_t local_count = local_end - local_begin;
    
    if (local_count < 3) return {local_count, NAN, NAN, NAN};
    
    auto shape = range.to_vector();
    
    auto index0 = max_by(shape,
                         [target = *(local_end - 1)](point_t point) {
                             return -std::hypot(target.x - point.x,
                                                target.y - point.y);
                         }),
         index1 = max_by(shape,
                         [target = *local_begin](point_t point) {
                             return -std::hypot(target.x - point.x,
                                                target.y - point.y);
                         });
    
    if (index1 < index0) index1 += range.point_count();
    
    shape.resize(local_count + index1 - index0);
    std::copy(local_begin, local_end, shape.begin());
    for (auto i = local_count; i < shape.size(); ++i)
        shape[i] = range[(index0++) % range.point_count()];
    
    std::cout << any_shape(shape).size() << std::endl;
    
    return {
        local_count,
        any_shape(std::vector<point_t>(local_begin, local_end)).size(),
        std::hypot(local_begin->x - local_end->x,
                   local_begin->y - local_end->y),
        2 * (0.5 - any_shape(shape).size() / range.size())
    };
}


#endif //PM1_SDK_VIRTUAL_LIGHT_SENSOR_T_HPP
