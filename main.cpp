//
// Created by ydrml on 2019/2/22.
//

#include <iostream>
#include "src/test/test.h"
#include "src/main/pm1/time_extensions.h"

using namespace autolabor::pm1::test;

int main() {
	std::cout << std::endl
	          << std::endl
	          << mechdancer::common::measure_time([] {
		          for (auto i = 0; i < 100; ++i)
			          test_pack();
	          }).count()
	          << std::endl;
}
