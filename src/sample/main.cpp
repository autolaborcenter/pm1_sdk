#include <iostream>
#include "../main/pm1_sdk.h"
#include "../main/exception.h"

using namespace autolabor::pm1;

int main() {
	union_error_code code{};
	code.bits.ecu1_offline = true;
	std::cout << +code.code << std::endl;
	return 0;
}
