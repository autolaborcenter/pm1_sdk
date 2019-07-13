//
// Created by User on 2019/4/2.
//

#ifndef PM1_SDK_WEAK_LOCK_GUARD_HPP
#define PM1_SDK_WEAK_LOCK_GUARD_HPP


/**
 * 非阻塞区域锁
 * 
 * @tparam t 锁对象类型
 */
template<class t>
class weak_lock_guard {
    volatile bool own;
    t             &lock;

public:
    explicit weak_lock_guard(t &lock)
        : lock(lock), own(lock.try_lock()) {}
    
    ~weak_lock_guard() { if (own) lock.unlock(); }
    
    explicit operator bool() const { return own; }
    
    bool retry() { return own ? true : own = lock.try_lock(); }
};


#endif //PM1_SDK_WEAK_LOCK_GUARD_HPP
