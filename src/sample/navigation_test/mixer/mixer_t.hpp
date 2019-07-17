//
// Created by User on 2019/7/16.
//

#ifndef PM1_SDK_MIXER_T_HPP
#define PM1_SDK_MIXER_T_HPP


#include <deque>
#include <mutex>

#include "stamped_t.h"

namespace autolabor {
    template<class t1, class t2>
    class mixer_t {
        std::deque<stamped_t < t1>> queue1;
        std::deque<stamped_t < t2>> queue2;
    public:
        void push_back1(const stamped_t <t1> &data) { queue1.push_back(data); }
    
        void push_back2(const stamped_t <t2> &data) { queue2.push_back(data); }
    
        bool solve(t1 &master, t2 &helper) {
            auto _master = queue1.begin();
            auto _helper = queue2.begin() + 1;
            auto result  = false;
            std::cout << queue1.size() << ' ' << queue2.size() << std::endl;
            while (_master < queue1.end() && _helper < queue2.end()) {
                std::cout << "0" << std::endl;
                if (_master->time < (_helper - 1)->time) {
                    std::cout << "1" << std::endl;
                    ++_master;
                } else if (_master->time > _helper->time) {
                    std::cout << "2" << std::endl;
                    ++_helper;
                } else {
                    std::cout << "3" << std::endl;
                    auto temp1 = _master->time - (_helper - 1)->time,
                         temp2 = _helper->time - _master->time;
                    master = _master->value;
                    helper = temp1 < temp2
                             ? (_helper - 1)->value
                             : _helper->value;
                    result = true;
                    break;
                }
            }
            queue1.erase(queue1.begin(), std::min(queue1.end(), _master + 1));
            queue2.erase(queue2.begin(), std::max(queue2.begin(), _helper - 1));
            return result;
        }
    };
}


#endif //PM1_SDK_MIXER_T_HPP
