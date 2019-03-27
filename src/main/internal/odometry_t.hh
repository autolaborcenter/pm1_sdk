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
	namespace pm1 {
		/** 里程计更新信息 */
		template<class time_unit = autolabor::seconds_floating>
		struct delta_odometry_t { double left, rigth; time_unit time; };
		
		/** 里程计信息 */
		struct odometry_t {
			const double width;
			
			double s, x, y, theta, vx, vy, w;
			
			odometry_t operator+(const delta_odometry_t<> &) const;
			
			odometry_t &operator=(const odometry_t &);
			
			void clear();
		};
	}
}


#endif //PM1_SDK_ODOMETRY_T_HH
