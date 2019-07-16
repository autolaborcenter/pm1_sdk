//
// Created by User on 2019/7/16.
//

#ifndef PM1_SDK_MOBILE_BEACON_T_HH
#define PM1_SDK_MOBILE_BEACON_T_HH


#include <memory>
#include <chrono>
#include <thread>
#include <condition_variable>

#include "utilities/time_extensions.h"
#include "utilities/serial_port/serial_port.hh"
#include "utilities/serial_parser/parse_engine.hpp"

#include "parser_t.hpp"

namespace marvelmind {
    class mobile_beacon_t {
        using engine_t   = autolabor::parse_engine_t<parser_t>;
        using word_t     = typename engine_t::word_t;
        using serial_ptr = std::unique_ptr<serial_port>;
        
        serial_ptr port;
        engine_t   engine;
        word_t     buffer[256]{};
    public:
        explicit mobile_beacon_t(const std::string &port_name);
    
        ~mobile_beacon_t();
        
        mobile_beacon_t(const mobile_beacon_t &) = delete;
    
        mobile_beacon_t(mobile_beacon_t &&) noexcept;
    
        mobile_beacon_t &operator=(const mobile_beacon_t &) = delete;
    
        mobile_beacon_t &operator=(mobile_beacon_t &&) noexcept;
    
        void receive(const typename engine_t::callback_t &);
    };
    
    mobile_beacon_t find_beacon(const std::string &port_name = "");
}


#endif //PM1_SDK_MOBILE_BEACON_T_HH
