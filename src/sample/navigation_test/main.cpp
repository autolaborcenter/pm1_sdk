//
// Created by User on 2019/7/3.
//

#include "../../main/pm1_sdk_native.h"

extern "C" {
#include "../../main/internal/control_model/model.h"
}

#include "pid/shape_t.hpp"
#include "pid/functions.hpp"
#include "pid/path_manage.hpp"
#include "pid/virtual_light_sensor_t.hpp"

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
    
            auto path                   = load_path("path.txt");
            auto local_begin            = path.begin();
    
            virtual_light_sensor_t sensor({0.2, 0}, 0.2);
    
            double theta, ignore, speed = 0.4;
            while (true) {
                using namespace std::chrono_literals;
                
                double tx, ty;
                native::get_odometry(
                    ignore, ignore,
                    tx, ty, theta,
                    ignore, ignore, ignore);
    
                // 模拟单光感
                auto last_begin = local_begin;
                auto local_end  = path.end();
                auto result     = sensor({tx, ty}, theta, local_begin, local_end);
                std::cout << "index = " << local_begin - path.begin()
                          << ", local count  = " << result.local_count
                          << ", error = " << result.error << std::endl;
    
                if (result.tip_begin) {
                    if (local_begin == path.end() - 1)
                        break;
                    
                    native::drive_physical(0, NAN);
                    std::this_thread::sleep_for(100ms);
        
                    native::get_odometry(
                        ignore, ignore,
                        tx, ty, theta,
                        ignore, ignore, ignore);
                    auto error = std::atan2((local_begin + 1)->y - local_begin->y,
                                            (local_begin + 1)->x - local_begin->x) - theta;
                    while (error > +pi_f) error -= 2 * pi_f;
                    while (error < -pi_f) error += 2 * pi_f;
        
                    native::drive_spatial(0, error > 0 ? 1 : -1, 0, error, ignore);
                    native::adjust_rudder(0, ignore);
                    ++local_begin;
                    continue;
                }
    
                if (std::isnan(result.error)) break;
    
                native::drive_physical(
                    speed = std::min(speed + 0.1, 2.0),
                    -pi_f / 2 * result.error);
                
                std::this_thread::sleep_for(100ms);
    
                plot << tx << ' ' << ty << std::endl;
                plot.flush();
            }
            plot.close();
        }
            break;
    }
    
    return 0;
}
