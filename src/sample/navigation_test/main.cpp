//
// Created by User on 2019/7/3.
//

#include "pm1_sdk_native.h"
#include "path_follower/path_manage.hpp"
#include "path_follower/path_follower_t.hpp"

#include <iostream>
#include <filesystem>

#define MARVELMIND

#ifdef MARVELMIND

#include <thread>
#include "marvelmind/mobile_beacon_t.hh"

#endif

enum operation_t : uint8_t {
    record,
    navigate
} this_time;

void odometry_simple(double &x, double &y, double &theta) {
    double ignore;
    autolabor::pm1::native::get_odometry(
        ignore, ignore,
        x, y, theta,
        ignore, ignore, ignore);
}

constexpr auto
    path_file       = "path.txt",
    navigation_file = "navigation.txt",
    marvelmind_file = "marvelmind.txt";

constexpr auto
    step = 0.05;

int main() {
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
        std::cout << "connected" << std::endl;
    }
    
    #ifdef MARVELMIND
    // 连接定位标签
    decltype(marvelmind::find_beacon()) beacon;
    try { beacon = marvelmind::find_beacon(); }
    catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    #endif
    
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
        this_time =
            command == "record"
            ? operation_t::record
            : operation_t::navigate;
    }
    
    #ifdef MARVELMIND
    std::thread([&] {
        std::filesystem::remove(marvelmind_file);
        std::fstream plot(marvelmind_file, std::ios::out);
    
        std::vector<marvelmind::telementry_t> space;
        while (true) {
            beacon->fetch(space);
            std::cout << space.size() << std::endl;
        }
    }).detach();
    #endif
    
    switch (this_time) {
        case operation_t::record: { // 记录路径
            std::filesystem::remove(path_file);
            std::fstream plot(path_file, std::ios::out);
            
            size_t count = 0;
            double x, y, ignore;
            odometry_simple(x, y, ignore);
            plot << x << ' ' << y << std::endl;
            while (true) {
                using namespace std::chrono_literals;
                double tx, ty;
                odometry_simple(tx, ty, ignore);
                if (std::abs(tx - x) + std::abs(ty - y) > step) {
                    plot << (x = tx) << ' ' << (y = ty) << std::endl;
                    std::cout << "count = " << count++ << std::endl;
                }
                std::this_thread::sleep_for(100ms);
                plot.flush();
            }
        }
            break;
        case operation_t::navigate: { // 进行导航
            std::filesystem::remove(navigation_file);
            std::fstream plot(navigation_file, std::ios::out);
            // 加载路径
            auto         path = path_follower::load_path(path_file);
            std::cout << "path length = " << path.size() << std::endl;
            // 加载控制器
            path_follower::path_follower_t<decltype(path)>
                controller(.2, .0, .2);
            using state_t = typename decltype(controller)::following_state_t;
            // 初始化
            controller.set_path(path.begin(), path.end());
            native::set_command_enabled(true);
            
            auto finish = false;
            while (!finish) {
                using namespace std::chrono_literals;
                
                double x, y, theta, ignore;
                odometry_simple(x, y, theta);
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
                        std::cout << "stopped" << std::endl;
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
