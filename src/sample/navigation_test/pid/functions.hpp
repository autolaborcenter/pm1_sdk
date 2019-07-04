//
// Created by User on 2019/7/4.
//

#ifndef PM1_SDK_FUNCTIONS_HPP
#define PM1_SDK_FUNCTIONS_HPP

#include <vector>

template<class t, class f>
std::vector<t> take_once(const std::vector<t> &source,
                         const f &function) {
    auto begin = source.begin(),
         end   = source.end();
    while (begin < end) {
        if (function(*begin)) {
            end = begin + 1;
            break;
        }
        ++begin;
    }
    for (; end < source.end() && function(*end); ++end);
    std::vector<t> result(end - begin);
    std::copy(begin, end, result.begin());
    return result;
}

template<class t, class f>
size_t max_by(const std::vector<t> &source,
              const f &function) {
    size_t max_index = 0;
    auto   max_value = function(source.front());
    
    for (size_t i = 1; i < source.size(); ++i) {
        auto value = function(source[i]);
        if (value > max_value) {
            max_index = i;
            max_value = value;
        }
    }
    
    return max_index;
}

#endif //PM1_SDK_FUNCTIONS_HPP
