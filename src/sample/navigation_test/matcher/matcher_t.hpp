//
// Created by User on 2019/7/16.
//

#ifndef PM1_SDK_MATCHER_T_HPP
#define PM1_SDK_MATCHER_T_HPP


#include <deque>
#include <mutex>

#include "stamped_t.h"

namespace autolabor {
    /**
     * 时序匹配器
     * 非线程安全
     * @tparam master_t 主配类型
     * @tparam helper_t 匹配类型
     */
    template<class master_t, class helper_t>
    class matcher_t {
        std::deque<stamped_t<master_t>> queue1;
        std::deque<stamped_t<helper_t>> queue2;
        
        seconds_floating max_error;
    public:
        /**
         * 构造器
         * @param max_error 匹配项的最大时间偏差
         */
        explicit matcher_t(seconds_floating max_error = std::chrono::milliseconds(100))
            : max_error(max_error) {}
        
        /** 向主配队列添加元素 */
        void push_back_master(const stamped_t<master_t> &data) { queue1.push_back(data); }
        
        /** 向匹配队列增加元素 */
        void push_back_helper(const stamped_t<helper_t> &data) { queue2.push_back(data); }
        
        /**
         * 执行一次匹配
         * 找到一对匹配项或无法找到匹配项时退出
         * 匹配项满足：
         * * 主配元素时刻在一对匹配元素时刻之间
         * * 匹配元素在整个匹配列表中与主配元素最接近
         */
        bool match(master_t &master, helper_t &helper) {
            //            std::cout << "masters:" << std::endl;
            //            for (const auto &item : queue1)
            //                std::cout << duration_seconds<double>(item.time.time_since_epoch()) << std::endl;
            //
            //            std::cout << "helpers:" << std::endl;
            //            for (const auto &item : queue2)
            //                std::cout << duration_seconds<double>(item.time.time_since_epoch()) << std::endl;
            
            auto _master = queue1.begin();     // 主配元素迭代器，每一个都能匹配
            auto _helper = queue2.begin() + 1; // 匹配元素迭代器，至少要有两个
            auto result  = false;
            while (_master < queue1.end() && _helper < queue2.end()) {
                if (_master->time < (_helper - 1)->time)
                    ++_master;
                else if (_master->time > _helper->time)
                    ++_helper;
                else {
                    auto duration0 = _master->time - (_helper - 1)->time,
                         duration1 = _helper->time - _master->time;
                    if ((result = duration0 < max_error && duration1 < max_error)) {
                        auto t0 = duration_seconds<double>(duration0),
                             t1 = duration_seconds<double>(duration1);
                        
                        // 主配元素一旦使用就要被消耗掉，绝不反复出现
                        master = _master++->value;
                        helper = (t1 * (_helper - 1)->value + t0 * _helper->value) / (t0 + t1);
                        break;
                    } else
                        ++_master;
                }
            }
            queue1.erase(queue1.begin(), _master);
            queue2.erase(queue2.begin(), _helper - 1);
            return result;
        }
    };
}


#endif //PM1_SDK_MATCHER_T_HPP
