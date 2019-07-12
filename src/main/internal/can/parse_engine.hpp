//
// Created by User on 2019/7/11.
//

#ifndef SERIAL_PIPELINE_PARSE_ENGINE_HPP
#define SERIAL_PIPELINE_PARSE_ENGINE_HPP


#include <functional>
#include "circular_buffer.hpp"

namespace autolabor {
    template<class parser_t>
    struct parse_engine_t {
        using word_t   = typename parser_t::word_t;
        using result_t = typename parser_t::result_t;
        
        template<class iterator_t>
        void operator()(
            iterator_t begin,
            iterator_t end,
            const std::function<void(const result_t &)> &callback) {
            buffer.resize(buffer.size() + end - begin);
            std::copy(begin, end, buffer.begin());
            
            auto parse_begin = buffer.begin(),
                 parse_end   = buffer.end();
            
            while (true) {
                auto _end = parse_end;
                callback(parser(parse_begin, _end));
                while (buffer.begin() < parse_begin) buffer.pop_front();
                if (_end == parse_end) return;
            }
        }
    
    private:
        circular_buffer<word_t> buffer{};
        parser_t                parser;
    };
} // namespace autolabor


#endif //SERIAL_PIPELINE_PARSE_ENGINE_HPP
