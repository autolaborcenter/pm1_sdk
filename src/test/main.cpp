//
// Created by ydrml on 2019/2/22.
//

#include <iostream>
#include "../main/pm1_sdk.h"

using namespace autolabor::pm1;

int main() {
	initialize("com4");
	std::cerr << std::boolalpha << (bool) go_straight(-.2, .8) << std::endl;
}
