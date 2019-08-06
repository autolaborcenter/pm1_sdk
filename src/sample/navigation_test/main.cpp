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

#include "navigation_system_t.hh"

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
    try {
        using namespace autolabor;
        using namespace autolabor::pm1;
    
        std::cout << "Hello world!" << std::endl;
        navigation_system_t system;
        
        { // 读取指令
            std::string command;
            std::cout << "input operation: ";
            std::cin >> command;
            operation =
                command == "record"
                ? operation_t::record
                : operation_t::navigate;
        }
        
        switch (operation) {
            case operation_t::record: { // 记录路径
                volatile auto flag   = true;
                auto          thread = std::thread([&] {
                    std::error_code _noexcept;
                    std::filesystem::remove(path_file, _noexcept);
                    std::fstream recorder(path_file, std::ios::out);
    
                    odometry_t<> memory = ODOMETRY_INIT;
                    while (flag) {
                        auto location = system.locate();
                        bool next;
                        if (std::isnan(memory.x))
                            next = true;
                        else {
                            auto dx = memory.x - location.x,
                                 dy = memory.y - location.y;
                            next = dx * dx + dy * dy > step * step;
                        }
                        if (next) {
                            memory = location;
                            recorder << memory.x << ' ' << memory.y << std::endl;
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
                auto path = path_follower::load_path(path_file);
                std::cout << "path length = " << path.size() << std::endl;
                
                // 加载控制器
                path_follower::path_follower_t<decltype(path)>
                    controller(.3, .0, .25);
                using state_t = typename decltype(controller)::following_state_t;
                
                // 初始化
                controller.set_path(path.begin(), path.end());
                native::set_command_enabled(true);
                
                auto finish = false;
                while (!finish) {
                    using namespace std::chrono_literals;
                    
                    auto pose = system.locate();
                    
                    auto result = controller(pose.x, pose.y, pose.theta);
                    
                    switch (result.type()) {
                        case state_t::following:
                            std::cout << "following: " << result.speed << std::endl;
                            native::drive_physical(result.speed, result.rudder);
                            break;
                        case state_t::turning:
                            std::cout << "turning" << std::endl;
                            native::drive_physical(0, NAN);
                            std::this_thread::sleep_for(100ms);
                            {
                                double ignore;
                                native::drive_spatial(0, result.rudder > 0 ? 0.4 : -0.4,
                                                      0, result.rudder,
                                                      ignore);
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
    } catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
    }
    
    return 0;
}
