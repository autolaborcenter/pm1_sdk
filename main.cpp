//
// Created by ydrml on 2019/2/22.
//

#include <iostream>
#include "src/test/test.h"
#include "src/main/pm1/time_extensions.h"
#include "src/main/pm1/extensions.h"

using namespace autolabor::pm1::test;

class t {
public:
	t() {
		std::cout << "Hello world!" << std::endl;
	}
	
	~t() {
		std::cout << "Goodbye world!" << std::endl;
	}
};

int main() {
	auto a = std::make_shared<t>();
	a = std::make_shared<t>();
	mechdancer::common::sleep(1000);
	std::cout << (a == nullptr) << std::endl;
	a = nullptr;
	std::cout << (a == nullptr) << std::endl;
}
