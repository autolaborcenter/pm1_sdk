#include <iostream>
#include <string>
#include <chrono>
#include "pm1_sdk.h"

int main() {
    constexpr static auto pi = 3.141592653589793238462643383279502884f;
    std::string           _;
    #pragma clang diagnostic push
    #pragma ide diagnostic ignored "EndlessLoop"
    for (size_t times = 1; times < 1000; ++times) {
        std::cout << "test " << times << ": press enter to launch the test";
        while ('\n' != std::getchar());
        std::cout << "test is running, search the robot" << std::endl;
        
        auto result = autolabor::pm1::initialize();
        if (result) {
            using namespace autolabor::pm1;
            using namespace std::chrono;
            
            std::cout << result.value << std::endl;
            unlock();
            delay(0.1);
            
            std::cout << "test left";
            for (auto i = 0; i < 20; ++i) {
                drive_wheels(pi, 0);
                delay(.5);
                std::cout << '.';
            }
            drive_physical(0, NAN);
            delay(.5);
            std::cout << std::endl;
            
            std::cout << "test right";
            for (auto i = 0; i < 20; ++i) {
                drive_wheels(0, pi);
                delay(.5);
                std::cout << '.';
            }
            drive_physical(0, NAN);
            delay(.5);
            std::cout << std::endl;
            
        } else
            std::cerr << result.error_info << std::endl;
        autolabor::pm1::shutdown();
        autolabor::pm1::delay(.5);
        std::cout << std::endl;
    }
    #pragma clang diagnostic pop
}
