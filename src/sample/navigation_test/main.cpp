//
// Created by User on 2019/7/3.
//

#include <string>
#include <iostream>
#include <filesystem>
#include <chrono>

#include "pm1_sdk_native.h"
#include "path_follower/path_manage.hpp"
#include "path_follower/path_follower_t.hpp"
#include "telementry_t.h"

#include <thread>
#include "marvelmind/mobile_beacon_t.hh"
#include "mixer/matcher_t.hpp"
#include "mixer/fusion_locator_t.hpp"

enum operation_t : uint8_t {
    record,
    navigate
} operation;

constexpr auto
    path_file       = "path.txt",
    navigation_file = "navigation.txt";

constexpr auto
    step = 0.05;

int main() {
    using namespace autolabor;
    using namespace autolabor::pm1;
    
    { // native sdk 连接串口
        double progress;
        auto   handler = native::initialize("", progress);
        auto   error   = std::string(native::get_error_info(handler));
        if (!error.empty()) {
            native::remove_error_info(handler);
            std::cout << error << std::endl;
            return 1;
        }
        std::cout << "chassis connected" << std::endl;
    }
    decltype(marvelmind::find_beacon()) beacon;
    { // 连接定位标签
        try { beacon = marvelmind::find_beacon(); }
        catch (std::exception &e) {
            std::cerr << e.what() << std::endl;
            return 1;
        }
        std::cout << "beacon connected" << std::endl;
    }
    { // 设置参数、修改状态
        native::set_parameter(0, 0.465);
        native::set_parameter(1, 0.355);
        native::set_parameter(2, 0.105);
        
        native::set_enabled(true);
        native::set_command_enabled(false);
    }
    { // 读取指令
        std::string command;
        std::cout << "input operation: ";
        std::cin >> command;
        operation =
            command == "record"
            ? operation_t::record
            : operation_t::navigate;
    }
    
    autolabor::fusion_locator_t<50> locator;
    const auto                      locate = [&] {
        pose_t pose{};
        { // 取定位数据
            using data_t = typename marvelmind::mobile_beacon_t::stamped_data_t;
            std::vector<data_t> temp;
            beacon->fetch(temp);
            for (auto item : temp) locator.push_back_master(item);
        }
        { // 取里程计数据
            double ignore;
            autolabor::pm1::native::get_odometry(
                ignore, ignore,
                pose.x, pose.y, pose.theta,
                ignore, ignore, ignore);
            locator.push_back_helper({autolabor::now(), {pose.x, pose.y}});
        }
        locator.refresh();
        return locator[pose];
    };
    
    switch (operation) {
        case operation_t::record: { // 记录路径
            volatile auto flag   = true;
            auto          thread = std::thread([&] {
                std::error_code _noexcept;
                std::filesystem::remove(path_file, _noexcept);
                std::fstream recorder(path_file, std::ios::out);
    
                size_t size = 0;
                pose_t pose = invalid_pose;
                while (flag) {
                    auto location = locate();
                    bool next;
                    if (std::isnan(pose.x))
                        next = true;
                    else {
                        auto dx = pose.x - location.x,
                             dy = pose.y - location.y;
                        next = dx * dx + dy * dy > step * step;
                    }
                    if (next) {
                        pose = location;
                        recorder << pose.x << ' ' << pose.y << std::endl;
                    }
    
                    recorder.flush();
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
                recorder.close();
            });
    
            std::string command;
            do std::cin >> command;
            while (command != "stop");
            flag = false;
            thread.join();
        }
            break;
        case operation_t::navigate: { // 进行导航
            std::error_code _noexcept;
            std::filesystem::remove(navigation_file, _noexcept);
            std::fstream plot(navigation_file, std::ios::out);
            // 加载路径
            auto         path = path_follower::load_path(path_file);
            std::cout << "path length = " << path.size() << std::endl;
    
            for (auto item:path)
                std::cout << item.x << ' ' << item.y << ' ' << +item.tip_order << std::endl;
            
            // 加载控制器
            path_follower::path_follower_t<decltype(path)>
                controller(.2, .0, .25);
            using state_t = typename decltype(controller)::following_state_t;
            // 初始化
            controller.set_path(path.begin(), path.end());
            native::set_command_enabled(true);
            
            auto finish = false;
            while (!finish) {
                using namespace std::chrono_literals;
    
                auto pose = locate();
                std::cout << pose.x << ' ' << pose.y << std::endl;
    
                auto result = controller(pose.x, pose.y, pose.theta);
                
                switch (result.type()) {
                    case state_t::following:
                        native::drive_physical(result.speed, result.rudder);
                        break;
                    case state_t::turning:
                        std::cout << "turning" << std::endl;
                        native::drive_physical(0, NAN);
                        std::this_thread::sleep_for(100ms);
                        {
                            double ignore;
                            native::drive_spatial(0, result.rudder > 0 ? 1 : -1,
                                                  0, result.rudder,
                                                  ignore);
                            native::adjust_rudder(0, ignore);
                        }
                        break;
                    case state_t::failed:
                        std::cerr << "following failed" << std::endl;
                    case state_t::finish:
                        std::cout << "stopped" << std::endl;
                        finish = true;
                        break;
                }
    
                plot << pose.x << ' ' << pose.y << std::endl;
                plot.flush();
                
                std::this_thread::sleep_for(50ms);
            }
            plot.close();
        }
            break;
    }
    
    return 0;
}
