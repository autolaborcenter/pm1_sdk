//
// Created by ydrml on 2019/2/25.
//

#include "chassis.hh"

autolabor::pm1::chassis::~chassis() {
	port.close();
}