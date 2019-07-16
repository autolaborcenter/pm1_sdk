//
// Created by User on 2019/7/16.
//

#include "protocol.h"

#define DEFINE_ELEMENT(NAME, TYPE, OFFSET) \
TYPE marvelmind::resolution_coordinate::NAME(const uint8_t *begin) { return *(TYPE *) (begin + (OFFSET)); }

DEFINE_ELEMENT(time_stamp, uint32_t, 0)

DEFINE_ELEMENT(x, int32_t, 4)

DEFINE_ELEMENT(y, int32_t, 8)

DEFINE_ELEMENT(z, int32_t, 12)

DEFINE_ELEMENT(flags, uint8_t, 16)

DEFINE_ELEMENT(address, uint8_t, 17)

DEFINE_ELEMENT(pair_direction, uint16_t, 18)

DEFINE_ELEMENT(time_passed, uint16_t, 20)
