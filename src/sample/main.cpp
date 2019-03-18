//
// Created by ydrml on 2019/2/22.
//

#include <thread>
#include <iostream>

extern "C" {
#include <math.h>
#include "../main/internal/control_model/model.h"
}

int main() {
	physical x{1, 0};
	physical_to_wheels(&x, &default_config);
	// std::cout << atanf(-0.317 / -0.21);
}
