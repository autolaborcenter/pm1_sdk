//
// Created by ydrml on 2019/2/22.
//

#include <iostream>
#include "src/test/test.h"
#include "src/main/pm1/time_extensions.h"
#include "src/main/pm1/extensions.h"
#include "src/main/pm1/api.h"

using namespace autolabor::pm1::test;

void run(const std::function<void()> &block) {
	block();
}

int x = 100;

int main() {
	autolabor::pm1::initialize("com3");
}
