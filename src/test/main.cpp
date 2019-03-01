//
// Created by ydrml on 2019/2/22.
//

#include <iostream>
#include "../main/pm1_sdk.h"
#include "../main/internal/mechanical.h"

using namespace autolabor::pm1;

int main() {
	initialize();
	turn_around(-.2, mechanical::pi / 2);
	// go_straight(.2, 2);
}
