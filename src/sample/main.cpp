#include <iostream>
#include "../main/pm1_sdk.h"
#include "../main/exception.h"

using namespace autolabor::pm1;

int main() {
	initialize();
	go_straight(-0.1, 0.3);
	std::cout << "Hello world!" << std::endl;
	return 0;
}
