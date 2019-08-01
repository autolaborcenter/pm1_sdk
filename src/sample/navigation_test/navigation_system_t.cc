//
// Created by User on 2019/7/19.
//

#include "navigation_system_t.hh"

#include <thread>
#include <iostream>
#include "pm1_sdk_native.h"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

autolabor::pm1::navigation_system_t::navigation_system_t(
    size_t locator_queue_size,
    double step)
    : locator(locator_queue_size, step),
      beacon(marvelmind::find_beacon()),
      particle_filter(16) {
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
    
    std::thread([this] {
        odometry_t<> save{};
        while (true) {
            auto _now = now();
            
            odometry_t<> odometry{};
            native::get_odometry(odometry.s, odometry.a, odometry.x, odometry.y, odometry.theta);
            if (std::abs(odometry.s - save.s) < 0.05) continue;
            
            save = odometry;
            
            using data_t = typename marvelmind::mobile_beacon_t::stamped_data_t;
            std::vector<data_t> temp;
            beacon->fetch(temp);
            
            if (temp.empty()) continue;
            particle_filter.update(odometry, temp.back().value);
            std::cout << "update!" << std::endl;
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
    
    odometry_t<> odometry{};
    native::get_odometry(odometry.s, odometry.a, odometry.x, odometry.y, odometry.theta);
    std::cout << duration_seconds<double>(now().time_since_epoch()) << ' '
              << odometry.x << ' '
              << odometry.y << ' '
              << odometry.theta << ' '
              << std::endl;
    
    return pose_t();
}

#pragma clang diagnostic pop
