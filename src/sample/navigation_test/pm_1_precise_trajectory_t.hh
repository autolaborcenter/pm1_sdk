//
// Created by User on 2019/7/3.
//

#ifndef PM1_SDK_PM_1_PRECISE_TRAJECTORY_T_HH
#define PM1_SDK_PM_1_PRECISE_TRAJECTORY_T_HH

#include <stdexcept>
#include "trajectory.hpp"

extern "C" {
#include "../../main/internal/control_model/chassis_config_t.h"
#include "../../main/internal/control_model/model.h"
}

struct pm1_precise_trajectory_t final
    : public sample_trajectory_t {
    
    pm1_precise_trajectory_t(
        chassis_config_t config,
        physical state,
        _duration_t period
    );
    
    [[nodiscard]] velocity get_state(int index) const {
        auto time = get_time(index);
        
    }
    
    [[nodiscard]] with_time_t<pose_t> operator[](int index) const final {
        if (index < 0)throw std::logic_error("index should be positive");
    }

private:
    chassis_config_t config;
    physical         state;
    float            rudder_omega    = pi_f / 2.5f;
    float            max_wheel_speed = pi_f * 3.5f;
    float            max_v           = 1.1f;
    float            max_w           = pi_f / 4;
    float            optimize_width  = pi_f / 4;
    float            acceleration    = max_wheel_speed;
};


#endif //PM1_SDK_PM_1_PRECISE_TRAJECTORY_T_HH
