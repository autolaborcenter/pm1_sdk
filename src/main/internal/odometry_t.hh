//
// Created by ydrml on 2019/3/12.
//

#ifndef PM1_SDK_ODOMETRY_T_HH
#define PM1_SDK_ODOMETRY_T_HH


#include "time_extensions.h"

extern "C" {
#include "control_model/chassis_config_t.h"
}

namespace autolabor {
	/** 里程计信息 */
	struct odometry_t {
		double s, x, y, theta, vx, vy, w;
		
		odometry_t operator+(const odometry_t &) const;
		
		odometry_t operator-(const odometry_t &) const;
		
		void operator+=(const odometry_t &delta) { *this = *this + delta; }
		
		void operator-=(const odometry_t &delta) { *this = *this - delta; }
		
		/** 清空里程 */
		void clear();
	};
	
	/** 里程计更新信息 */
	struct delta_differential_t {
		double           width, left, rigth;
		seconds_floating time;
		
		explicit operator odometry_t();
	};
}


#endif //PM1_SDK_ODOMETRY_T_HH
