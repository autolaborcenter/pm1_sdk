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
            // 连接到解析缓冲区
            buffer.insert(buffer.end(), begin, end);
            // 初始化迭代器
            decltype(buffer.begin())
                parse_begin = buffer.begin(),
                parse_end;
            // 解析到全部已检查
            do {
                parse_end = buffer.end();
                callback(parser(parse_begin, parse_end));
            } while (parse_end < buffer.end());
            // 清除已解析部分
            buffer.erase(buffer.begin(), parse_begin);
        }
    
    private:
        std::deque<word_t> buffer{};
        parser_t           parser;
    };
} // namespace autolabor


#endif //SERIAL_PIPELINE_PARSE_ENGINE_HPP
