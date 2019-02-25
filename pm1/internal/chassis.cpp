//
// Created by ydrml on 2019/2/25.
//

#include "chassis.h"

autolabor::pm1::chassis *const autolabor::pm1::chassis::instance() {
	static chassis instance;
	return &instance;
}
