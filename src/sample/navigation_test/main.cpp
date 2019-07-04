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
#include "pid/virtual_light_sensor_t.hh"

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
                if (std::abs(tx - x) + std::abs(ty - y) < 0.05)
                    continue;
                plot << (x = tx) << ' ' << (y = ty) << std::endl;
                std::cout << "count = " << count++ << std::endl;
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
    
            auto path        = load_path("path.txt");
            auto local_begin = path.begin(),
                 local_end   = path.end();
    
            virtual_light_sensor_t sensor;
            
            double x, y, theta, ignore;
            native::get_odometry(
                ignore, ignore,
                x, y, ignore,
                ignore, ignore, ignore);
            while (true) {
                using namespace std::chrono_literals;
                std::this_thread::sleep_for(100ms);
                
                double tx, ty;
                native::get_odometry(
                    ignore, ignore,
                    tx, ty, theta,
                    ignore, ignore, ignore);
                
                constexpr static auto width = 0.41f;
    
                sensor.set_pose(
                    point_t{tx + width / 2 * std::cos(theta),
                            ty + width / 2 * std::sin(theta)
                    }, theta);
    
                local_end = path.end();
                auto error = sensor(local_begin, local_end);
                std::cout << "error = " << error << ", ";
                if (!std::isfinite(error)) break;
    
                auto speed  = 2.0,
                     rudder = -pi_f / 2 * error;
                std::cout << "speed = " << speed << ", rudder = " << rudder << std::endl;
                native::drive_physical(speed, rudder);
    
                if (std::abs(tx - x) + std::abs(ty - y) > 0.05)
                    plot << (x = tx) << ' ' << (y = ty) << std::endl;
            }
            plot.flush();
            plot.close();
        }
            break;
    }
    
    return 0;
}
