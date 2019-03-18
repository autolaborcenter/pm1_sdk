//
// Created by ydrml on 2019/3/18.
//

#ifndef PM1_SDK_SPEED_CONTROLLER_HH
#define PM1_SDK_SPEED_CONTROLLER_HH


namespace autolabor {
	namespace pm1 {
		class speed_controller {
		public:
			const double max_speed,
			             min_speed,
			             dead_area;
			const double k;
			
			speed_controller(double k, double dead_area, double min_speed, double max_speed);
			
			double operator()(double error) const;
		};
	}
}


#endif //PM1_SDK_SPEED_CONTROLLER_HH
