//
// Created by User on 2019/7/11.
//

#ifndef SERIAL_PIPELINE_PARSE_ENGINE_HPP
#define SERIAL_PIPELINE_PARSE_ENGINE_HPP


#include <functional>
#include <deque>

namespace autolabor {
    /**
     * 解析引擎
     * 为任意解析器提供回溯
     * @tparam parser_t 解析器类型
     */
    template<class parser_t>
    struct parse_engine_t {
        using word_t     = typename parser_t::word_t;
        using result_t   = typename parser_t::result_t;
        using callback_t = std::function<void(const result_t &)>;
        
        template<class iterator_t>
        void operator()(iterator_t begin,
                        iterator_t end,
                        const callback_t &callback) {
            
            buffer.resize(buffer.size() + end - begin);
            std::copy(begin, end, buffer.begin());
            
            parse(callback);
        }
    
    private:
        std::deque<word_t> buffer{};
        parser_t           parser;
        
        void parse(const callback_t &callback) {
            while (true) {
                auto begin = buffer.begin(),
                     end   = buffer.end();
                
                callback(parser(begin, end));
                buffer.erase(buffer.begin(), begin);
                if (buffer.empty() || end == buffer.end()) return;
            }
        }
    };
} // namespace autolabor


#endif //SERIAL_PIPELINE_PARSE_ENGINE_HPP
