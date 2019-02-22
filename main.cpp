//
// Created by ydrml on 2019/2/22.
//

#include <iostream>
#include "pm1/sdk/Api.h"

template<class T>
void println(T data) {
	std::cout << data << std::endl;
}

int main() {
	pm1::sdk::Pause();
	println(1);
	pm1::sdk::TurnAroundTiming(0, 1);
	println(2);
	pm1::sdk::Delay(1);
	println(3);
}
