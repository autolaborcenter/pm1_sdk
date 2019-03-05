//
// Created by ydrml on 2019/2/22.
//

#include <iostream>
#include <random>
#include "../main/pm1_sdk.h"
#include "../main/internal/mechanical.h"

using namespace autolabor::pm1;

int main() {
	mechanical::state t1(100, 100);
	auto              t2 = mechanical::state::from_wheels(t1.left, t1.right, t1.theta);
	std::cout << t2.rho << std::endl
	          << t2.theta << std::endl;
}
