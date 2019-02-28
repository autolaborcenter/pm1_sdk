//
// Created by ydrml on 2019/2/22.
//

#include <iostream>
#include "../main/pm1_sdk.h"

using namespace autolabor::pm1;

int main() {
	auto success = go_straight(1, 10000);
	if (!success) std::cerr << success.error_info << std::endl;
}
