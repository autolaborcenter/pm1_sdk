//
// Created by ydrml on 2019/3/14.
//

#ifndef PM1_SDK_CHASSIS_T_H
#define PM1_SDK_CHASSIS_T_H


const float pi_f;

struct chassis_t {
	int wheel_encoder_resolution,
	    rudder_encoder_resolution;
	
	float wheel_mapping,
	      rudder_mapping,
	//----------------
	      width,
	      length,
	      diameter,
	      radius,
	//----------------
	      max_wheel_speed,
	      max_rudder_speed,
	//----------------
	      max_v,
	      max_w;
} const     default_chassis;

#define _chassis_of(_wheel_encoder_resolution, _rudder_encoder_resolution, _width, _length, _wheel_diameter, _max_wheel_speed, _max_rudder_speed) \
{                                                          \
.wheel_encoder_resolution  = (_wheel_encoder_resolution),  \
.rudder_encoder_resolution = (_rudder_encoder_resolution), \
                                                           \
.wheel_mapping  = 2 * pi_f / (_wheel_encoder_resolution),  \
.rudder_mapping = 2 * pi_f / (_rudder_encoder_resolution), \
                                                           \
.width    = (_width),                                      \
.length   = (_length),                                     \
.diameter = (_wheel_diameter),                             \
.radius   = (_wheel_diameter) / 2,                         \
                                                           \
.max_wheel_speed  = (_max_wheel_speed),                    \
.max_rudder_speed = (_max_rudder_speed),                   \
                                                           \
.max_v = (_max_wheel_speed) * (_wheel_diameter) / 2,       \
.max_w = (_max_wheel_speed) * (_wheel_diameter) / (_width) \
}


#endif //PM1_SDK_CHASSIST_H
