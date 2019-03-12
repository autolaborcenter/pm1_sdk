//
// Created by ydrml on 2019/3/12.
//

#ifndef PM1_SDK_RUDDER_CONTROLLER_HH
#define PM1_SDK_RUDDER_CONTROLLER_HH


#include "serial/serial.h"

namespace autolabor {
	namespace pm1 {
		class rudder_controller {
		public:
			explicit rudder_controller(const std::string &);
			
			void adjust(short);
			
			void done();
		
		private:
			short          target;
			serial::Serial port;
		};
	}
}


#endif //PM1_SDK_RUDDER_CONTROLLER_HH
