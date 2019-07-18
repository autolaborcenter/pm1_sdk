//
// Created by User on 2019/7/17.
//

#ifndef PM1_SDK_TELEMENTRY_T_H
#define PM1_SDK_TELEMENTRY_T_H


#include <cmath>

namespace autolabor {
    struct telementry_t { double x, y; };
    struct pose_t { double x, y, theta; };
    const pose_t invalid_pose{NAN, NAN, NAN};
}


#endif //PM1_SDK_TELEMENTRY_T_H
