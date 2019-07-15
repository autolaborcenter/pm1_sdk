//
// Created by User on 2019/7/3.
//

#include "../../main/pm1_sdk_native.h"
#include "pid/path_manage.hpp"
#include "pid/virtual_light_sensor_t.hpp"
#include "pid/path_follower_t.hpp"

#include <iostream>
#include <filesystem>
#include <conio.h>

enum operation : uint8_t {
    record,
    navigate
} this_time;

int main() {
    using namespace autolabor::pm1;
    
    {
        double progress;
        auto   handler = native::initialize("", progress);
        auto   error   = std::string(native::get_error_info(handler));
        if (!error.empty()) {
            native::remove_error_info(handler);
            std::cout << error << std::endl;
            return 1;
        }
        std::cout << "connected" << std::endl;
    }
    
    native::set_parameter(0, 0.470);
    native::set_parameter(1, 0.345);
    native::set_parameter(2, 0.105);
    
    native::set_enabled(true);
    native::set_command_enabled(false);
    
    std::string command;
    std::cout << "input operation: ";
    std::cin >> command;
    this_time =
        command == "record"
        ? operation::record
        : operation::navigate;
    
    switch (this_time) {
        case operation::record: {
            constexpr static auto filename = "path.txt";
            std::filesystem::remove(filename);
            std::fstream plot(filename, std::ios::out);
            
            size_t count = 0;
            double x, y, ignore;
            native::get_odometry(
                ignore, ignore,
                x, y, ignore,
                ignore, ignore, ignore);
            plot << x << ' ' << y << std::endl;
            while (!_kbhit()) {
                using namespace std::chrono_literals;
                std::this_thread::sleep_for(100ms);
                double tx, ty;
                native::get_odometry(
                    ignore, ignore,
                    tx, ty, ignore,
                    ignore, ignore, ignore);
                if (std::abs(tx - x) + std::abs(ty - y) > 0.05) {
                    plot << (x = tx) << ' ' << (y = ty) << std::endl;
                    std::cout << "count = " << count++ << std::endl;
                }
            }
            plot.flush();
            plot.close();
        }
            break;
        case operation::navigate: {
            native::set_command_enabled(true);
            
            constexpr static auto filename = "navigation.txt";
            std::filesystem::remove(filename);
            std::fstream plot(filename, std::ios::out);
    
    
            // 资源
            auto path = load_path("path.txt");
    
            path_follower_t<decltype(path)> controller(.2, .0, .2);
            using state_t = typename decltype(controller)::following_state_t;
            
            controller.set_path(path.begin(), path.end());
            auto finish = false;
            while (!finish) {
                using namespace std::chrono_literals;
                
                double x, y, theta, ignore;
                native::get_odometry(
                    ignore, ignore,
                    x, y, theta,
                    ignore, ignore, ignore);
    
                auto result = controller(x, y, theta);
    
                switch (result.type()) {
                    case state_t::following:
                        native::drive_physical(result.speed, result.rudder);
                        break;
                    case state_t::turning:
                        std::cout << "turning" << std::endl;
                        native::drive_physical(0, NAN);
                        std::this_thread::sleep_for(100ms);
            
                        native::drive_spatial(0, result.rudder > 0 ? 1 : -1,
                                              0, result.rudder,
                                              ignore);
                        native::adjust_rudder(0, ignore);
                        break;
                    case state_t::failed:
                        std::cerr << "following failed" << std::endl;
                    case state_t::finish:
                        finish = true;
                        break;
                }
                
                std::this_thread::sleep_for(100ms);
                
                plot << x << ' ' << y << std::endl;
                plot.flush();
            }
            plot.close();
        }
            break;
    }
    
    return 0;
}
