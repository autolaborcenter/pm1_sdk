//
// Created by ydrml on 2019/5/31.
//

#ifndef PM1_SDK_CHASSIS_STATE_T_HH
#define PM1_SDK_CHASSIS_STATE_T_HH


#include "can_define.h"

namespace autolabor {
    namespace pm1 {
        /** 底盘状态 */
        struct chassis_state_t {
            constexpr static auto size = 4;
            
            union {
                struct { node_state_t _ecu0, _ecu1, _tcu, _vcu; };
                node_state_t states[size];
            };
            
            class iterator {
                chassis_state_t &master;
            
            public:
                int i;
                
                iterator(chassis_state_t &master, int i);
                
                iterator(const iterator &others) = default;
                
                iterator &operator=(const iterator &others);
                
                bool operator!=(const iterator &others) const;
    
                bool operator==(const iterator &others) const;
    
                iterator operator++();
    
                const iterator operator++(int);
                
                node_state_t operator*() const;
            };
            
            iterator begin();
            
            iterator end();
        };
    }
}


#endif //PM1_SDK_CHASSIS_STATE_T_HH
