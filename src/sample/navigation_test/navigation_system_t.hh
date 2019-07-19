//
// Created by User on 2019/7/19.
//

#ifndef PM1_SDK_NAVIGATION_SYSTEM_T_HH
#define PM1_SDK_NAVIGATION_SYSTEM_T_HH


#include "marvelmind/mobile_beacon_t.hh"
#include "matcher/fusion_locator_t.hh"

namespace autolabor {
    namespace pm1 {
        class navigation_system_t {
            decltype(marvelmind::find_beacon()) beacon;
            autolabor::fusion_locator_t         locator;
        
        public:
            explicit navigation_system_t(size_t locator_queue_size = 30);
            
            bool refresh();
            
            pose_t locate();
        };
    }
}


#endif //PM1_SDK_NAVIGATION_SYSTEM_T_HH
