//
// Created by ydrml on 2019/3/20.
//

#ifndef PM1_SDK_MOTOR_MAP_H
#define PM1_SDK_MOTOR_MAP_H

#include "model.h"

#define CALCULATE_K(RESOLUTION, RATIO) (4 * (RESOLUTION) * (RATIO))

#define RADS_OF(PULSES, K) ((int)((PULSES) / (K)))
#define PULSES_OF(RADS, K) ((RADS) * (K))

const float default_wheel_k = 2 * PI_F / CALCULATE_K(400, 20);

const float default_rudder_k = 2 * PI_F / -16384;

#endif //PM1_SDK_MOTOR_MAP_H
