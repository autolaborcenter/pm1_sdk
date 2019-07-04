//
// Created by User on 2019/7/4.
//

#include "virtual_light_sensor_t.hh"

#include "functions.hpp"

virtual_light_sensor_t::virtual_light_sensor_t()
    : range(16, 0.2) {}

void virtual_light_sensor_t::set_pose(point_t position, double direction) {
    range.center    = position;
    range.direction = direction;
}
