//
// Created by User on 2019/7/3.
//

#ifndef PM1_SDK_PM1_TRAJECTORY_T_HH
#define PM1_SDK_PM1_TRAJECTORY_T_HH


#include "trajectory.hpp"

struct pm1_sample_trajectory_t;

struct pm1_trajectory_t final : public continuous_trajectory_t {
    pm1_trajectory_t(double v, double w);
    
    [[nodiscard]] pose_t operator[](_duration_t duration) const final;
    
    [[nodiscard]] typename sample_trajectory_t::ptr_t
    sample(_duration_t period) const final;

private:
    double v, w, r;
};

struct pm1_sample_trajectory_t final : public sample_trajectory_t {
    using ptr_t = std::shared_ptr<pm1_sample_trajectory_t>;
    
    explicit pm1_sample_trajectory_t(
        typename pm1_trajectory_t::ptr_t trajectory,
        _duration_t period
    );
    
    [[nodiscard]] with_time_t<pose_t> operator[](int index) const final;

private:
    typename pm1_trajectory_t::ptr_t trajectory;
};


#endif //PM1_SDK_PM1_TRAJECTORY_T_HH
