//
// Created by User on 2019/4/9.
//

#ifndef PM1_SDK_EXCEPTION_ENGINE_HPP
#define PM1_SDK_EXCEPTION_ENGINE_HPP


#include <shared_mutex>
#include <unordered_map>

namespace autolabor {
    /**
     * 异常驱动器
     * 线程安全
     */
    template<class handler_t = unsigned int>
    class exception_engine {
        mutable std::shared_mutex                  mutex;
        std::unordered_map<handler_t, std::string> map;
    
    public:
        /**
         * 增、改异常信息
         * @param id  操作序列号
         * @param msg 异常信息
         */
        void set(handler_t id, const std::string &msg) {
            std::unique_lock<decltype(mutex)> _(mutex);
            if (msg.empty())
                map.erase(id);
            else
                map[id] = msg;
        }
        
        /**
         * 移除异常信息
         * @param id 操作序列号
         */
        void remove(handler_t id) {
            std::unique_lock<decltype(mutex)> _(mutex);
            map.erase(id);
        }
        
        /**
         * 清除异常信息
         */
        void clear() {
            std::unique_lock<decltype(mutex)> _(mutex);
            map.clear();
        }
        
        /**
         * 根据序列号读取异常信息
         * @param id 操作序列号
         * @return 异常信息
         */
        const char *operator[](handler_t id) const {
            std::shared_lock<decltype(mutex)> _(mutex);
            
            auto ptr = map.find(id);
            return ptr == map.end() ? "" : ptr->second.c_str();
        }
    };
}


#endif //PM1_SDK_EXCEPTION_ENGINE_HPP
