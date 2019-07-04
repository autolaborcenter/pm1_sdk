//
// Created by User on 2019/7/3.
//

#include "../../main/pm1_sdk_native.h"

extern "C" {
#include "../../main/internal/control_model/model.h"
}

#include "pid/shape.hpp"
#include "pid/functions.hpp"
#include <iostream>

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
    std::cout << "connected" << std::endl;
    
    
    
    
    return 0;
    //
    //    native::set_enabled(true);
    //
    //    double width, length, radius;
    //    using id_enum = autolabor::pm1::parameter_id;
    //
    //    native::get_parameter(static_cast<uint8_t>(id_enum::width), width);
    //    native::get_parameter(static_cast<uint8_t>(id_enum::length), length);
    //    native::get_parameter(static_cast<uint8_t>(id_enum::wheel_radius), radius);
    //
    //    const chassis_config_t config{
    //        static_cast<float>(width),
    //        static_cast<float>(length),
    //        static_cast<float>(radius)};
    //
    //    const auto speed  = 2 * pi_f,
    //               rudder = 1.0f;
    //
    //    auto velocity = physical_to_velocity({speed, rudder}, &config);
    //    auto temp     = pm1_trajectory_t(velocity.v, velocity.w);
    //
    //    native::set_command_enabled(false);
    //    std::this_thread::sleep_for(std::chrono::hours(1));
    //
    //    const auto begin = std::chrono::steady_clock::now();
    //    double     x, y, theta, _rudder;
    //    while (true) {
    //        native::drive_physical(speed, rudder);
    //        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    //        auto   now     = std::chrono::steady_clock::now();
    //        auto   seconds = std::chrono::duration_cast<
    //            std::chrono::duration<double, std::ratio<1>>
    //        >(now - begin).count();
    //        auto   pre     = temp[now - begin];
    //        double ignore;
    //        native::get_odometry(ignore, ignore,
    //                             x, y, theta,
    //                             ignore, ignore, ignore);
    //        native::get_rudder(_rudder);
    //        std::cout << seconds << ' '
    //                  << pre.x << ' '
    //                  << pre.y << ' '
    //                  << x << ' '
    //                  << y << ' '
    //                  << _rudder << std::endl;
    //
    //        if (seconds > 5) {
    //            native::shutdown();
    //            return 0;
    //        }
    //    }
}
