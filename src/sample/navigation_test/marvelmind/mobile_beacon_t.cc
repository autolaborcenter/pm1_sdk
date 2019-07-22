//
// Created by User on 2019/7/16.
//

#include "mobile_beacon_t.hh"

#include <algorithm>
#include <sstream>
#include <thread>
#include <condition_variable>

#include <fstream>
#include <filesystem>

#include "utilities/serial_port/serial.h"
#include "utilities/serial_parser/parse_engine.hpp"

using engine_t = autolabor::parse_engine_t<marvelmind::parser_t>;
using word_t   = typename engine_t::word_t;

marvelmind::mobile_beacon_t::
mobile_beacon_t(const std::string &port_name)
    : port(std::make_unique<serial_port>(port_name, 115200)),
      running(std::make_shared<bool>(true)) {
    std::condition_variable signal;
    
    std::thread([&, _running = running] {
        std::filesystem::remove("marvelmind.txt");
        std::ofstream plot("marvelmind.txt", std::ios::out);
        
        auto       first_time = true;
        const auto function   =
                       [&](const typename engine_t::result_t &result) {
                           if (!*_running || result.type != parser_t::result_type_t::success)
                               return;
                
                           using namespace marvelmind::resolution_coordinate;
                           auto begin = result.bytes.data() + 5;
                           auto delay = time_passed(begin);
                           plot << x(begin) / 1000.0 << ' '
                                << y(begin) / 1000.0 << ' '
                                << delay << std::endl;
                           plot.flush();
                           if (delay > 1000) return;
                
                           std::lock_guard<decltype(buffer_mutex)> lk(buffer_mutex);
                           buffer.push_back({
                                                autolabor::now() - std::chrono::milliseconds(delay),
                                                {x(begin) / 1000.0, y(begin) / 1000.0}
                                            });
                
                           if (first_time) {
                               first_time = false;
                               signal.notify_all();
                           }
                       };
        
        engine_t _engine;
        word_t   _buffer[64]{};
        while (*_running) _engine(_buffer, _buffer + port->read(_buffer, sizeof(_buffer)), function);
    }).detach();
    
    using namespace std::chrono_literals;
    
    std::mutex                   signal_mutex;
    std::unique_lock<std::mutex> own(signal_mutex);
    if (signal.wait_for(own, 1s, [&] { return !buffer.empty(); }))
        return;
    
    *running = false;
    port->break_read();
    throw std::runtime_error("no position data until timeout");
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
