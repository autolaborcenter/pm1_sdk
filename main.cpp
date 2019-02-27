//
// Created by ydrml on 2019/2/22.
//

#include <iostream>
#include "src/test/test.h"
#include "src/main/pm1/time_extensions.h"
#include "src/main/pm1/extensions.h"
#include "src/main/pm1/api.h"
#include "src/main/pm1/internal/can/can_message.h"
#include "src/main/pm1/internal/can/can_define.h"

using namespace autolabor::pm1;
using namespace autolabor::pm1::test;

int main() {
	autolabor::pm1::initialize("com4");
	autolabor::pm1::go_arc(1, 1, 3.14);
}
