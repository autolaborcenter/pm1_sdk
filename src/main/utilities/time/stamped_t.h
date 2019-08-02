//
// Created by User on 2019/7/16.
//

#ifndef PM1_SDK_STAMPED_T_H
#define PM1_SDK_STAMPED_T_H


#include "time_extensions.h"

namespace autolabor {
    /**
     * 时刻数据
     * @tparam t
     */
    template<class t>
    struct stamped_t {
        decltype(now()) time;
        t               value;
    };
    
    template<class time_t, class t>
    stamped_t<t> make_stamped(const t &value, time_t time = now()) {
        return {time, value};
    }
}


#endif //PM1_SDK_STAMPED_T_H
