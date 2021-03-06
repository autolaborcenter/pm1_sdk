﻿//
// Created by User on 2019/4/2.
//

#ifndef PM1_SDK_WEAK_SHARED_LOCK_HPP
#define PM1_SDK_WEAK_SHARED_LOCK_HPP


#include <shared_mutex>

/**
 * 非阻塞的共享区域锁
 *
 * 只在构造时尝试获取锁一次，失败也立即退出
 */
class weak_shared_lock {
    volatile bool     own;
    std::shared_mutex &lock;

public:
    explicit weak_shared_lock(std::shared_mutex &lock)
        : lock(lock), own(lock.try_lock_shared()) {}
    
    ~weak_shared_lock() { if (own) lock.unlock_shared(); }
    
    explicit operator bool() const { return own; }
};


#endif //PM1_SDK_WEAK_SHARED_LOCK_HPP
