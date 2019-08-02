//
// Created by User on 2019/7/19.
//

#include "navigation_system_t.hh"

#include <thread>
#include <iostream>
#include <filesystem>
#include "pm1_sdk_native.h"

using namespace std::chrono_literals;

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

autolabor::pm1::navigation_system_t::navigation_system_t(
    size_t locator_queue_size,
    double step)
    : locator(locator_queue_size, step),
      beacon(marvelmind::find_beacon()),
      matcher(250ms),
      particle_filter(32) {
    // native sdk 连接串口
    double progress;
    auto   handler = native::initialize("", progress);
    auto   error   = std::string(native::get_error_info(handler));
    if (!error.empty()) {
        native::remove_error_info(handler);
        throw std::runtime_error(error);
    }
    // 设置参数、修改状态
    native::set_parameter(0, 0.465);
    native::set_parameter(1, 0.355);
    native::set_parameter(2, 0.105);
    
    native::set_enabled(true);
    native::set_command_enabled(false);
    
    std::error_code _;
    std::filesystem::remove("locator.txt", _);
    plot = std::ofstream("locator.txt", std::ios::out);
    
    std::thread([this] {
        while (true) {
            double       time_stamp;
            odometry_t<> odometry{};
            native::get_odometry_stamped(time_stamp,
                                         odometry.s, odometry.a,
                                         odometry.x, odometry.y, odometry.theta);
    
            decltype(now()) _time_stamp{};
            _time_stamp += std::chrono::duration_cast<std::chrono::milliseconds>(seconds_duration(time_stamp));
            matcher.push_back_helper({_time_stamp, Eigen::Vector3d{odometry.x, odometry.y, odometry.theta}});
            
            using data_t = typename marvelmind::mobile_beacon_t::stamped_data_t;
            std::vector<data_t> buffer;
            beacon->fetch(buffer);
    
            for (const auto &item : buffer) matcher.push_back_master(item);
    
            auto time = measure_time([&] {
                Eigen::Vector2d target;
                Eigen::Vector3d source;
                while (matcher.match(target, source)) {
                    auto result = particle_filter.update(
                        odometry_t<>{0, 0, source[0], source[1], source[2]},
                        Eigen::Vector2d{target[1], target[0]}
                    );
                    plot << result.x << ' ' << result.y << ' ' << result.theta << std::endl;
                    plot.flush();
                    std::cout << result.x << ' ' << result.y << ' ' << result.theta << std::endl;
                }
            });
    
            if (time > 1ms)
                std::cout << "time = " << 1000 * duration_seconds<double>(time) << std::endl
                          << "----------------------------------------------" << std::endl;
        }
    }).detach();
}

autolabor::pose_t autolabor::pm1::navigation_system_t::locate() {
    //    using data_t = typename marvelmind::mobile_beacon_t::stamped_data_t;
    //    std::vector<data_t> temp;
    //    beacon->fetch(temp);
    //    for (const auto &item : temp) {
    //        locator.push_back_master(item);
    //        matcher.push_back_master(item);
    //    }
    //
    //    auto _now = now();
    //
    //    odometry_t<> odometry{};
    //    native::get_odometry(odometry.s, odometry.a, odometry.x, odometry.y, odometry.theta);
    //    locator.push_back_helper({_now, {odometry.x, odometry.y}});
    //    matcher.push_back_helper({_now, odometry});
    //    locator.refresh();
    //
    //    odometry_t<>    source{};
    //    Eigen::Vector2d target;
    //    while (matcher.match(target, source)) {
    //        std::cout << duration_seconds<double>(_now.time_since_epoch()) << std::endl;
    //        particle_filter.update(source, target);
    //    }
    
    return pose_t();
}

#pragma clang diagnostic pop
