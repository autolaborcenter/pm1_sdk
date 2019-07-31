//
// Created by User on 2019/7/19.
//

#include "navigation_system_t.hh"

#include <thread>
#include "pm1_sdk_native.h"

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
    
    using namespace std::chrono_literals;
    while (true) {
        locate();
        std::this_thread::sleep_for(50ms);
    }
}

autolabor::pose_t autolabor::pm1::navigation_system_t::locate() {
    odometry_t<> odometry{};
    native::get_odometry(odometry.s, odometry.a, odometry.x, odometry.y, odometry.theta);
    locator.push_back_helper({autolabor::now(), {odometry.x, odometry.y}});
    matcher.push_back_helper({autolabor::now(), odometry});
    
    using data_t = typename marvelmind::mobile_beacon_t::stamped_data_t;
    std::vector<data_t> temp;
    beacon->fetch(temp);
    for (const auto &item : temp) {
        locator.push_back_master(item);
        matcher.push_back_master(item);
    }
    locator.refresh();
    
    odometry_t<>    source{};
    Eigen::Vector2d target;
    while (matcher.match(target, source))
        particle_filter.update(source, target);
    
    auto result = particle_filter(odometry);
    return locator[{odometry.x, odometry.y, odometry.theta}];
}
