//
// Created by User on 2019/5/30.
//

#ifndef PM1_SDK_CHASSIS_MAP_HPP
#define PM1_SDK_CHASSIS_MAP_HPP


#include <shared_mutex>
#include <unordered_map>

#include "../raii/safe_shared_ptr.hpp"
#include "../chassis.hh"

namespace autolabor {
    template<class handler_t = unsigned int>
    class chassis_map {
        mutable std::shared_mutex
            mutex;
        std::unordered_map<handler_t, safe_shared_ptr<autolabor::pm1::chassis>>
            map;
    };
}


#endif //PM1_SDK_CHASSIS_MAP_HPP
