#include <iostream>
#include "../main/pm1_sdk.h"
#include "../main/exception.h"

using namespace autolabor::pm1;

int main() {
	auto result = initialize();
	if (!result) {
		std::cout << result.error_info << std::endl;
		return 1;
	}
	go_straight(-0.1, 0.3);
	std::cout << "Hello world!" << std::endl;
	return 0;
}
