//
// Created by ydrml on 2019/2/22.
//

#include <iostream>
#include "../main/pm1_sdk.h"

using namespace autolabor::pm1;

int main() {
	auto temp = initialize("com4");
	if (!temp) std::cerr << temp.error_info << std::endl;
	else go_arc(1, 1, 1);
}
