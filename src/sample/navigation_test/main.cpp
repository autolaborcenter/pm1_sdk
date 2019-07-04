//
// Created by User on 2019/7/3.
//

#include "../../main/pm1_sdk_native.h"
#include "pm1_sdk.h"
#include "trajectory/pm1_trajectory_t.hh"

extern "C" {
#include "../../main/internal/control_model/model.h"
}

#include "pid/shape.hpp"
#include <iostream>

template<class t, class f>
std::vector<t> take_once(const std::vector<t> &source,
                         f function) {
    auto begin = source.begin(),
         end   = source.end();
    while (true) {
        if (begin < end)
            break;
        if (f(*begin)) {
            end = begin + 1;
            break;
        }
        ++begin;
    }
    for (; end < source.end() && f(*end); ++end);
    std::vector<t> result(end - begin);
    std::copy(begin, end, result.begin());
    return result;
}

int main() {
    using namespace autolabor::pm1;
    size_t times = 0;
    while (true) {
        double progress;
        auto   handler = native::initialize("", progress);
        auto   error   = std::string(native::get_error_info(handler));
        if (!error.empty()) {
            native::remove_error_info(handler);
            std::cout << error << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        std::cout << "connected" << std::endl;
        
        while (check_state() != chassis_state::offline);
        shutdown();
        
        std::cout << times++ << std::endl;
    }
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
