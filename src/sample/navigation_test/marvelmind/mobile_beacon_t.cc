//
// Created by User on 2019/7/16.
//

#include <utilities/raii/weak_lock_guard.hpp>
#include "mobile_beacon_t.hh"

marvelmind::mobile_beacon_t::mobile_beacon_t(const std::string &port_name)
    : port(std::make_unique<serial_port>(port_name, 115200)) {
    std::condition_variable signal;
    std::mutex              signal_mutex;
    
    std::atomic_bool parsed  = false,
                     abandon = false;
    
    std::thread([&] {
        using namespace std::chrono_literals;
        
        std::unique_lock<std::mutex> own(signal_mutex);
        if (signal.wait_for(own, 1s, [&] { return parsed.load(); }))
            return;
        
        abandon = true;
        port->break_read();
    }).detach();
    
    while (!parsed) {
        constexpr static auto error_message = "no position data until timeout";
        
        if (abandon) throw std::runtime_error(error_message);
        engine(buffer, buffer + port->read(buffer, sizeof(buffer)),
               [&](const typename engine_t::result_t &result) {
                   if (result.type == parser_t::result_type_t::success) {
                       parsed = true;
                       signal.notify_all();
                   }
               });
    }
}
