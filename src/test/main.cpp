//
// Created by ydrml on 2019/2/22.
//

#include "../main/pm1/pm1_sdk.h"

using namespace autolabor::pm1;

int main() {
	initialize("com4");
	go_arc(1, 1, 1);
}
