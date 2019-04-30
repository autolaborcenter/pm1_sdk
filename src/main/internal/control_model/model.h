﻿//
// Created by ydrml on 2019/3/15.
//

#ifndef PM1_SDK_MODEL_H
#define PM1_SDK_MODEL_H

#include "chassis_config_t.h"

struct physical { float speed, rudder; }; // 物理模型
// speed  := 较快轮的角速度，
// rudder := 后轮转角
struct wheels { float left, right; };     // 两轮角速度（rad/s）
struct velocity { float v, w; };          // 速度空间（标准单位）

/** 物理空间 -> 差动轮速空间 */
struct wheels physical_to_wheels(
    const struct physical *,
    const struct chassis_config_t *);

/** 差动轮速空间 -> 物理空间 */
struct physical wheels_to_physical(
    const struct wheels *,
    const struct chassis_config_t *);

/** 物理空间 -> 速度矢量空间 */
struct velocity physical_to_velocity(
    const struct physical *,
    const struct chassis_config_t *);

/** 速度矢量空间 -> 物理空间 */
struct physical velocity_to_physical(
    const struct velocity *,
    const struct chassis_config_t *);

/** 速度矢量空间 -> 差动轮速空间 */
struct wheels velocity_to_wheels(
    const struct velocity *,
    const struct chassis_config_t *);

/** 差动轮速空间 -> 速度矢量空间 */
struct velocity wheels_to_velocity(
    const struct wheels *,
    const struct chassis_config_t *);

/** 在速度空间中限速 */
void limit_in_velocity(struct physical *,
                       float max_v,
                       float max_w,
                       const struct chassis_config_t *);

/** 在物理模型空间中限速 */
void limit_in_physical(struct physical *,
                       float);


#endif //PM1_SDK_MODEL_H
