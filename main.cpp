//
// Created by ydrml on 2019/2/22.
//

#include <thread>
#include <iostream>
#include "pm1/sdk/Api.h"
#include "pm1/sdk/TimeExtensions.h"

using namespace mechdancer::common;

template<class T>
void println(T data) {
	std::cout << data << std::endl;
}

template<class T1, class T2>
void println(std::chrono::duration<T1, T2> duration) {
	println(duration.count());
}

int main() {
	pm1::sdk::Pause();
	println(1);
	pm1::sdk::TurnAroundTiming(0, 1);
	println(2);
	println(MeasureTime([] { pm1::sdk::Delay(1); }));
	println(3);
}
