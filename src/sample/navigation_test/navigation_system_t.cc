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
    
    std::condition_variable signal;
    
    std::thread([&, this] {
        std::error_code _;
        std::filesystem::remove("locator.txt", _);
        auto plot       = std::ofstream("locator.txt", std::ios::out);
        auto first_time = true;
        
        while (true) {
            double       time_stamp;
            odometry_t<> odometry{};
            native::get_odometry_stamped(time_stamp,
                                         odometry.s, odometry.a,
                                         odometry.x, odometry.y, odometry.theta);
    
            decltype(now()) _time_stamp{};
            _time_stamp += std::chrono::duration_cast<std::chrono::milliseconds>(seconds_duration(time_stamp));
            matcher.push_back_helper({_time_stamp, Eigen::Vector3d{odometry.x, odometry.y, odometry.theta}});
    
            using data_t = marvelmind::mobile_beacon_t::stamped_data_t;
            std::vector<data_t> buffer;
            beacon->fetch(buffer);
    
            for (const auto &item : buffer) matcher.push_back_master(item);
    
            Eigen::Vector2d target;
            Eigen::Vector3d source;
            while (matcher.match(target, source)) {
                auto result = particle_filter.update(
                    odometry_t<>{0, 0, source[0], source[1], source[2]},
                    Eigen::Vector2d{target[1], target[0]});
                if (first_time)
                    std::cout << result.x << ' ' << result.y << ' ' << result.theta << std::endl;
                if (std::isnan(result.theta)) continue;
                plot << result.x << ' ' << result.y << ' ' << result.theta << std::endl;
                if (first_time) {
                    signal.notify_all();
                    first_time = false;
                }
            }
    
            std::this_thread::sleep_for(2ms);
        }
    }).detach();
    
    std::mutex                      _lk;
    std::unique_lock<decltype(_lk)> ulk(_lk);
    signal.wait(ulk);
}

autolabor::odometry_t<> autolabor::pm1::navigation_system_t::locate() {
    double       _;
    odometry_t<> odometry{};
    native::get_odometry_stamped(_,
                                 odometry.s, odometry.a,
                                 odometry.x, odometry.y, odometry.theta);
    return particle_filter(odometry);
}

#pragma clang diagnostic pop
