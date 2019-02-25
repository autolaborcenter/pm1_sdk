//
// Created by ydrml on 2019/2/25.
//

#ifndef PM1_SDK_PM1_H
#define PM1_SDK_PM1_H

namespace autolabor {
	namespace pm1 {
		
		class chassis {
		public:
			static chassis *const instance();
			
			chassis(const chassis &others) = delete;
			
			chassis(chassis &&others) = delete;
		
		private:
			chassis() = default;
		};
	}
}


#endif //PM1_SDK_PM1_H
