//
// Created by User on 2019/7/3.
//

#include "../../main/pm1_sdk_native.h"
#include "pm1_sdk.h"

extern "C" {
#include "../../main/internal/control_model/model.h"
}

#include <string>
#include <iostream>
#include <functional>
#include <chrono>
#include <thread>

struct pose_t {
    double x, y, theta;
};

/**
 * 计算轨迹
 *
 * @param config
 * @param _physical
 * @return
 */
std::function<pose_t(double)> trajectory(
    const chassis_config_t &config,
    physical _physical) {
    
    wheels _wheels = physical_to_wheels(_physical, &config);
    double v_left  = config.radius * _wheels.left,
           v_right = config.radius * _wheels.right,
           ds      = (v_right + v_left) / 2,
           da      = (v_right - v_left) / config.width,
           r       = ds / da;
    
    return [=](double seconds) {
        auto s = ds * seconds,
             a = da * seconds;
        return !std::isfinite(r)
               ? pose_t{s, 0, 0}
               : pose_t{r * std::sin(a),
                        r * (1 - std::cos(a)),
                        a};
    };
}

int main() {
    using namespace autolabor::pm1;
    
    double progress;
    auto   handler = native::initialize("", progress);
    auto   error   = std::string(native::get_error_info(handler));
    if (!error.empty()) {
        native::remove_error_info(handler);
        std::cout << error << std::endl;
        return 1;
    }
    
    native::set_enabled(true);
    
    double width, length, radius;
    using id_enum = autolabor::pm1::parameter_id;
    
    native::get_parameter(static_cast<uint8_t>(id_enum::width), width);
    native::get_parameter(static_cast<uint8_t>(id_enum::length), length);
    native::get_parameter(static_cast<uint8_t>(id_enum::wheel_radius), radius);
    
    const chassis_config_t config{
        static_cast<float>(width),
        static_cast<float>(length),
        static_cast<float>(radius)};
    
    auto speed  = 2 * pi_f,
         rudder = -1.2f;
    
    auto temp = trajectory(config, {speed, rudder});
    
    const auto begin = std::chrono::steady_clock::now();
    double     x, y, theta;
    while (true) {
        native::drive_physical(speed, rudder);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        auto   now     = std::chrono::steady_clock::now();
        auto   seconds = std::chrono::duration_cast<
            std::chrono::duration<double, std::ratio<1>>
        >(now - begin).count();
        auto   pre     = temp(seconds);
        double ignore;
        native::get_odometry(ignore, ignore,
                             x, y, theta,
                             ignore, ignore, ignore);
        std::cout << pre.x << ' '
                  << pre.y << ' '
                  << x << ' '
                  << y << ' ' << std::endl;
        
        if (seconds > 5) {
            native::shutdown();
            return 0;
        }
    }
}
