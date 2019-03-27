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
	/** 里程计更新信息 */
	template<class time_unit = autolabor::seconds_floating>
	struct delta_differential_t { double width, left, rigth; time_unit time; };
	
	/** 里程计信息 */
	struct odometry_t {
		double s, x, y, theta, vx, vy, w;
		
		/** 直接修改里程 */
		template<class delta_t>
		void operator+=(const delta_t &delta) { *this = *this + delta; }
		
		/** 清空里程 */
		void clear();
	};
}

/** 积分计算里程 */
autolabor::odometry_t operator+(const autolabor::odometry_t &,
                                const autolabor::delta_differential_t<> &);


#endif //PM1_SDK_ODOMETRY_T_HH
