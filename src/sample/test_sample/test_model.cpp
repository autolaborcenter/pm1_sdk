//
// Created by User on 2019/8/6.
//

#include <iostream>

extern "C" {
#include <internal/control_model/model.h>
}

int main() {
    auto rudder = -pi_f / 2;
    
    while (rudder <= pi_f / 2) {
        auto temp = physical_to_wheels({1, rudder}, &default_config);
        std::cout << 1 << ' ' << rudder << ' ' << temp.left << ' ' << temp.right << std::endl;
        rudder += 0.1;
    }
    return 0;
}
