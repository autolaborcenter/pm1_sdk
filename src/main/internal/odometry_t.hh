//
// Created by ydrml on 2019/3/12.
//

#ifndef PM1_SDK_ODOMETRY_T_HH
#define PM1_SDK_ODOMETRY_T_HH


#include "time_extensions.h"

namespace autolabor {
	namespace pm1 {
		/** 里程计更新信息 */
		template<class time_unit = autolabor::seconds_floating>
		struct odometry_update_info { double d_left, d_rigth; time_unit d_t; };
		
		/** 里程计信息 */
		struct odometry_t {
			double s, x, y, theta, vx, vy, w;
			
			void operator+=(const odometry_update_info<> &);
		};
	}
}


#endif //PM1_SDK_ODOMETRY_T_HH
