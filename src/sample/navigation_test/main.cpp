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
    
    std::string command;
    std::cout << "input operation: ";
    std::cin >> command;
    this_time =
        command == "record"
        ? operation::record
        : operation::navigate;
    
    switch (this_time) {
        case operation::record: {
            native::set_command_enabled(false);
            
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
            
            auto path = load_path("path.txt");
            
            double x, y, theta, ignore;
            native::get_odometry(
                ignore, ignore,
                x, y, ignore,
                ignore, ignore, ignore);
            plot << x << ' ' << y << std::endl;
            while (true) {
                using namespace std::chrono_literals;
                std::this_thread::sleep_for(100ms);
                
                double tx, ty;
                native::get_odometry(
                    ignore, ignore,
                    tx, ty, theta,
                    ignore, ignore, ignore);
                
                constexpr static auto width = 0.41f;
                
                point_t center{tx + width / 2 * std::cos(theta),
                               ty + width / 2 * std::sin(theta)};
    
                circle_t range(16, 0.2, center);
                
                auto local = take_once(path, [&](point_t point) {
                    return range.check_inside(point);
                });
                if (local.size() < 2) break;
                
                size_t begin, end;
                {
                    auto            vertex = range.to_vector();
                    begin = max_by(vertex,
                                   [target = local.back()](point_t point) {
                                       auto dx = target.x - point.x,
                                            dy = target.y - point.y;
                                       return -dx * dx - dy * dy;
                                   });
                    end   = max_by(vertex,
                                   [target = local.front()](point_t point) {
                                       auto dx = target.x - point.x,
                                            dy = target.y - point.y;
                                       return -dx * dx - dy * dy;
                                   });
                    if (end < begin) end += range.point_count();
                }
                
                std::cout << "local = " << local.size() << ", "
                          << "begin = " << begin << ", "
                          << "end = " << end << ", ";
                
                std::vector<point_t> vertex(local.size() + end - begin);
                std::copy(local.begin(), local.end(), vertex.begin());
                for (auto i = local.size(); i < vertex.size(); ++i)
                    vertex[i]   = range[(begin++) % range.point_count()];
                any_shape shape(vertex);
                auto      error = 2 * (0.5 - shape.size() / range.size());
                std::cout << "error = " << error << ", ";
    
                // auto v = 0.2 * (error + 1) / 2,
                //      w = 2 * error;
                // std::cout << "v = " << v << ", w = " << w << std::endl;
                // native::drive_velocity(v, w);
                auto speed  = 1.0,
                     rudder = -pi_f / 2 * error;
                std::cout << "speed = " << speed << ", rudder = " << rudder << std::endl;
                native::drive_physical(speed, rudder);
                
                if (std::abs(tx - x) + std::abs(ty - y) < 0.05)
                    continue;
                
                x = tx;
                y = ty;
                plot << x << ' ' << y << std::endl;
            }
            plot.flush();
            plot.close();
        }
            break;
    }
    
    return 0;
}
