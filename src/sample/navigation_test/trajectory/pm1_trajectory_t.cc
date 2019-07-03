//
// Created by User on 2019/7/3.
//

#include "pm1_trajectory_t.hh"

#include <cmath>

pm1_trajectory_t::pm1_trajectory_t(double v, double w)
    : v(v), w(w), r(v / w) {}

pose_t
pm1_trajectory_t::operator[](_duration_t duration) const {
    auto s = v * duration.count(),
         a = w * duration.count();
    return !std::isfinite(r)
           ? pose_t{s, 0, 0}
           : pose_t{r * std::sin(a),
                    r * (1 - std::cos(a)),
                    a};
}

typename sample_trajectory_t::ptr_t
pm1_trajectory_t::sample(_duration_t period) const {
    return std::make_shared<pm1_sample_trajectory_t>(
        std::make_shared<pm1_trajectory_t>(v, w),
        period
    );
}

pm1_sample_trajectory_t::pm1_sample_trajectory_t(
    typename pm1_trajectory_t::ptr_t trajectory,
    _duration_t period
) : sample_trajectory_t(period),
    trajectory(std::move(trajectory)) {}

with_time_t<pose_t>
pm1_sample_trajectory_t::operator[](int index) const {
    auto time = get_time(index);
    return with_time_t<pose_t>{trajectory->operator[](time), time};
}
