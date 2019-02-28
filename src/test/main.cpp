//
// Created by ydrml on 2019/2/22.
//

#include <iostream>
#include "../main/pm1_sdk.h"

using namespace autolabor::pm1;

int main() {
	auto result = initialize("com4");
	if (!result) std::cerr << result.error_info << std::endl;
	else {
		go_straight_timing(1, 10000);
	}
}
