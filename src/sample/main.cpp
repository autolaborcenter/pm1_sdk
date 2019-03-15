//
// Created by ydrml on 2019/2/22.
//

#include "adjust_rudder.h"

extern "C" {
#include "../main/internal/control_model/model.h"
}

int main() {
	for (float i = -pi_f / 2; i < pi_f / 2; i += 0.01f) {
		physical p{0.5, i};
		auto     result = physical_to_wheels(&p, &default_config);
		std::cout << result.left << ',' << result.right << std::endl;
	}
}
