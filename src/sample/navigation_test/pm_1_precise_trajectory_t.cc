//
// Created by User on 2019/7/3.
//

#include "pm_1_precise_trajectory_t.hh"

with_time_t<pose_t> pm1_precise_trajectory_t::operator[](int index) const {
    return with_time_t<pose_t>();
}

pm1_precise_trajectory_t::pm1_precise_trajectory_t(
    chassis_config_t config,
    physical state,
    _duration_t period
) : sample_trajectory_t(period),
    state(state),
    config(config) {}
