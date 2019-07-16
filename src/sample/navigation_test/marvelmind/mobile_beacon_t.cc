//
// Created by User on 2019/7/16.
//

#include "mobile_beacon_t.hh"

#include <algorithm>
#include <sstream>
#include <iostream>

#include "utilities/serial_port/serial.h"
#include "utilities/serial_parser/parse_engine.hpp"

using engine_t = autolabor::parse_engine_t<marvelmind::parser_t>;
using word_t   = typename engine_t::word_t;

marvelmind::mobile_beacon_t::
mobile_beacon_t(const std::string &port_name)
    : port(std::make_unique<serial_port>(port_name, 115200)),
      running(std::make_shared<bool>(true)) {
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
    
    engine_t _engine;
    word_t   _buffer[64]{};
    while (!parsed) {
        constexpr static auto error_message = "no position data until timeout";
        
        if (abandon) throw std::runtime_error(error_message);
        _engine(_buffer, _buffer + port->read(_buffer, sizeof(_buffer)),
                [&](const typename engine_t::result_t &result) {
                    if (result.type == parser_t::result_type_t::success) {
                        parsed = true;
                        signal.notify_all();
                    }
                });
    }
    
    std::thread([flag = running, this] {
        engine_t _engine;
        word_t   _buffer[64]{};
        while (*flag)
            _engine(_buffer, _buffer + port->read(_buffer, sizeof(_buffer)),
                    [&](const typename engine_t::result_t &result) {
                        if (result.type != parser_t::result_type_t::success)
                            return;
                
                        using namespace marvelmind::resolution_coordinate;
    
                        auto         begin = result.bytes.data() + 5;
                        telementry_t telementry{
                            autolabor::now(),
                            time_stamp(begin),
                            time_passed(begin),
                            x(begin) / 1000.0,
                            y(begin) / 1000.0,
                            z(begin) / 1000.0
                        };
    
                        if (telementry.time_passed > 250)
                            return;
    
                        std::lock_guard<decltype(buffer_mutex)> lk(buffer_mutex);
                        buffer.push_back(telementry);
                    });
    }).detach();
}

marvelmind::mobile_beacon_t::~mobile_beacon_t() {
    *running = false;
}

std::shared_ptr<marvelmind::mobile_beacon_t>
marvelmind::find_beacon(const std::string &port_name) {
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
            try { return std::make_shared<mobile_beacon_t>(*name); }
            catch (std::exception &e) {
                builder << *name << " : " << e.what();
                if (++name < list.end())
                    builder << std::endl;
                else
                    throw std::runtime_error(builder.str());
            }
    }
}
