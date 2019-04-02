#include <iostream>
#include "../pm1_sdk.h"

int main() {
	while (true) {
		auto result = autolabor::pm1::initialize();
		if (result)
			std::cout << result.data << std::endl;
		else
			std::cerr << result.error_info << std::endl;
		autolabor::pm1::shutdown();
	}
}

#pragma clang diagnostic pop
