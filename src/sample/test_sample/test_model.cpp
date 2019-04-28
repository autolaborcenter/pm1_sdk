#include <iostream>

extern "C" {
#include "../../main/internal/control_model/model.h"
}

int main() {
	velocity v{2, 0};
	auto     physical = velocity_to_physical(&v, &default_config);
	std::cout << physical.speed / pi_f << std::endl;
	return 0;
}
