//
// Created by User on 2019/7/16.
//

#ifndef PM1_SDK_MOBILE_BEACON_T_HH
#define PM1_SDK_MOBILE_BEACON_T_HH


#include <memory>
#include <mutex>
#include <deque>
#include <utilities/time/time_extensions.h>
#include <eigen3/Eigen/Core>

#include "utilities/serial_port/serial_port.hh"

#include "parser_t.hpp"

#include "../pose_t.h"
#include "utilities/time/stamped_t.h"

namespace marvelmind {
    /**
     * 移动标签类
     */
    struct mobile_beacon_t {
        using serial_ptr     = std::unique_ptr<serial_port>;
        using stamped_data_t = autolabor::stamped_t<Eigen::Vector2d>;
    private:
        serial_ptr port;
        
        std::shared_ptr<bool> running;
    
        std::deque<stamped_data_t> buffer;
        std::mutex                 buffer_mutex;
    public:
        explicit mobile_beacon_t(const std::string &port_name,
                                 int delay_ms);
        
        ~mobile_beacon_t();
        
        /**
         * 读取定位数据
         * @tparam t 容器类型
         * @param container 容器
         */
        template<class t>
        size_t fetch(t &container) {
            std::lock_guard<decltype(buffer_mutex)> lk(buffer_mutex);
    
            auto size = buffer.size();
            container.insert(container.end(), buffer.begin(), buffer.end());
            buffer.clear();
            return size;
        }
    };
    
    std::shared_ptr<mobile_beacon_t> find_beacon(
        const std::string &port_name = "",
        int delay_ms = 0);
}


#endif //PM1_SDK_MOBILE_BEACON_T_HH
