//
// Created by User on 2019/7/4.
//

#ifndef PM1_SDK_FUNCTIONS_HPP
#define PM1_SDK_FUNCTIONS_HPP

#include <vector>

template<class t, class f>
std::vector<t> take_once(const std::vector<t> &source,
                         f function) {
    auto begin = source.begin(),
         end   = source.end();
    while (true) {
        if (begin < end)
            break;
        if (f(*begin)) {
            end = begin + 1;
            break;
        }
        ++begin;
    }
    for (; end < source.end() && f(*end); ++end);
    std::vector<t> result(end - begin);
    std::copy(begin, end, result.begin());
    return result;
}


#endif //PM1_SDK_FUNCTIONS_HPP
