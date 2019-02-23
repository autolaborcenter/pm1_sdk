//
// Created by ydrml on 2019/2/22.
//

#include <thread>
#include <iostream>
#include "pm1/Extensions.h"
#include "pm1/TimeExtensions.h"
#include "pm1/Api.h"

using namespace mechdancer::common;

template<class _, class __>
inline void println(std::chrono::duration<_, __> duration) {
	println(duration.count());
}

#define fun int

fun main() {
	println(joinToString("", 1, 2, 3, 4, 5));
	println(joinToString(", ", 1, 2, 3, 4, 5));
	println(joinToString("", '[', joinToString(", ", 1, 2, 3, 4, 5), ']'));
}
