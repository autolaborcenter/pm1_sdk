//
// Created by User on 2019/7/3.
//

#ifndef PM1_SDK_TRAJECTORY_HPP
#define PM1_SDK_TRAJECTORY_HPP

#include <chrono>
#include <memory>
#include "pm1_sdk.h"

using _time_t     = decltype(std::chrono::steady_clock::now());
using _duration_t = std::chrono::duration<double, std::ratio<1>>;

struct pose_t { double x, y, theta; };

template<class t>
struct with_time_t {
    t           value;
    _duration_t time;
};

struct trajectory_t {
    _time_t reference_time;
};

struct discrete_trajectory_t : public trajectory_t {
    using ptr_t = std::shared_ptr<discrete_trajectory_t>;
    
    [[nodiscard]] virtual with_time_t<pose_t>
    operator[](int index) const = 0;
};

struct sample_trajectory_t : public discrete_trajectory_t {
    using ptr_t = std::shared_ptr<sample_trajectory_t>;
    
    _duration_t period;
    
    [[nodiscard]] _duration_t
    get_time(int index) const { return index * period; }
    
    explicit sample_trajectory_t(_duration_t period) : period(period) {}
};

struct continuous_trajectory_t : public trajectory_t {
    using ptr_t = std::shared_ptr<continuous_trajectory_t>;
    
    [[nodiscard]] virtual pose_t
    operator[](_duration_t duration) const = 0;
    
    [[nodiscard]] virtual typename sample_trajectory_t::ptr_t
    sample(_duration_t period) const = 0;
};

#endif //PM1_SDK_TRAJECTORY_HPP
