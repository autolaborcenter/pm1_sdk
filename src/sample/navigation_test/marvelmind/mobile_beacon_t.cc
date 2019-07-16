//
// Created by User on 2019/7/16.
//

#include "mobile_beacon_t.hh"

#include <algorithm>
#include <sstream>

#include "utilities/serial_port/serial.h"

marvelmind::mobile_beacon_t::
mobile_beacon_t(const std::string &port_name)
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

marvelmind::mobile_beacon_t::
~mobile_beacon_t() {}

marvelmind::mobile_beacon_t::
mobile_beacon_t(marvelmind::mobile_beacon_t &&others) noexcept
    : port(std::move(others.port)) {}

marvelmind::mobile_beacon_t &
marvelmind::mobile_beacon_t::
operator=(marvelmind::mobile_beacon_t &&others) noexcept {
    port = std::move(others.port);
    return *this;
}

void
marvelmind::mobile_beacon_t::
receive(typename marvelmind::mobile_beacon_t::engine_t::callback_t const &callback) {
    engine(buffer, buffer + port->read(buffer, sizeof(buffer)), callback);
}

marvelmind::mobile_beacon_t marvelmind::find_beacon(const std::string &port_name) {
    const static auto serial_ports = [] {
        auto                     info = serial::list_ports();
        std::vector<std::string> result(info.size());
        std::transform(info.begin(), info.end(), result.begin(),
                       [](const serial::PortInfo &it) { return it.port; });
        return result;
    };
    
    auto list = port_name.empty()
                ? serial_ports()
                : std::vector<std::string>{port_name};
    
    if (list.empty())
        throw std::runtime_error("no available port");
    else {
        std::stringstream builder;
        for (auto         name = list.begin();;)
            try { return mobile_beacon_t(*name); }
            catch (std::exception &e) {
                builder << *name << " : " << e.what();
                if (++name < list.end())
                    builder << std::endl;
                else
                    throw std::runtime_error(builder.str());
            }
    }
}
