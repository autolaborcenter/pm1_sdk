//
// Created by User on 2019/7/16.
//

#ifndef PM1_SDK_MOBILE_BEACON_T_HH
#define PM1_SDK_MOBILE_BEACON_T_HH


#include <memory>
#include <mutex>
#include <deque>
#include <utilities/time_extensions.h>

#include "utilities/serial_port/serial_port.hh"

#include "parser_t.hpp"

namespace marvelmind {
    /**
     * 定位数据
     */
    struct telementry_t {
        decltype(autolabor::now()) time;
        uint32_t                   time_stamp;
        uint16_t                   time_passed;
        double                     x, y, z;
    };
    
    /**
     * 移动标签类
     */
    class mobile_beacon_t {
        using serial_ptr = std::unique_ptr<serial_port>;
        
        serial_ptr port;
        
        std::shared_ptr<bool> running;
        
        std::deque<telementry_t> buffer;
        std::mutex               buffer_mutex;
    public:
        explicit mobile_beacon_t(const std::string &port_name);
        
        ~mobile_beacon_t();
        
        /**
         * 读取定位数据
         * @tparam t 容器类型
         * @param container 容器
         */
        template<class t>
        void fetch(t &container) {
            std::lock_guard<decltype(buffer_mutex)> lk(buffer_mutex);
            
            auto begin = buffer.begin(),
                 end   = buffer.end();
            container.insert(container.end(), begin, end);
            buffer.erase(begin, end);
        }
    };
    
    std::shared_ptr<mobile_beacon_t> find_beacon(const std::string &port_name = "");
}


#endif //PM1_SDK_MOBILE_BEACON_T_HH
