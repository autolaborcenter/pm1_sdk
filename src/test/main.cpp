//
// Created by ydrml on 2019/2/22.
//

#include <iostream>
#include <random>
#include "../main/pm1_sdk.h"
#include "../main/internal/mechanical.h"

using namespace autolabor::pm1;

int main() {
	mechanical::state t1(10, mechanical::pi / 2);
	auto              t2 = mechanical::state::from_wheels(t1.left, t1.right, t1.theta);
	std::cout << t2.rho << std::endl
	          << t2.theta << std::endl
	          << t1.left << std::endl
	          << t1.right << std::endl
	          << t2.left << std::endl
	          << t2.right << std::endl;
}
