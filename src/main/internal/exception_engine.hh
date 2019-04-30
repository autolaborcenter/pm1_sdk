//
// Created by User on 2019/4/9.
//

#ifndef PM1_SDK_EXCEPTION_ENGINE_HH
#define PM1_SDK_EXCEPTION_ENGINE_HH


#include <shared_mutex>
#include <unordered_map>

namespace autolabor {
    /**
     * 异常驱动器
     * 线程安全
     */
    class exception_engine {
        mutable std::shared_mutex               mutex;
        std::unordered_map<size_t, std::string> map;
    
    public:
        /**
         * 增、改异常信息
         * @param id  操作序列号
         * @param msg 异常信息
         */
        void set(size_t id, const std::string &msg);
        
        /**
         * 移除异常信息
         * @param id 操作序列号
         */
        void remove(size_t id);
        
        /**
         * 清除异常信息
         */
        void clear();
        
        /**
         * 根据序列号读取异常信息
         * @param id 操作序列号
         * @return 异常信息
         */
        const char *operator[](size_t id) const;
    };
}


#endif //PM1_SDK_EXCEPTION_ENGINE_HH
