//
// Created by User on 2019/4/1.
//

#include <iostream>
#include "pm1_sdk.h"

int main() {
	auto result = autolabor::pm1::initialize();
	if (result)
		std::cout << "connected to " << result.value
		          << ", which may be a pm1 chassis" << std::endl;
	else
		std::cerr << "initialize failed, because:" << std::endl
		          << result.error_info << std::endl;
	return 0;
}
