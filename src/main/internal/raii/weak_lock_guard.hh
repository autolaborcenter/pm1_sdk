//
// Created by User on 2019/4/2.
//

#ifndef PM1_SDK_WEAK_LOCK_GUARD_HH
#define PM1_SDK_WEAK_LOCK_GUARD_HH


template<class t>
class weak_lock_guard {
public:
	explicit weak_lock_guard(t &core)
			: core(core), own(core.try_lock()) {}
	
	~weak_lock_guard() {
		if (own) core.unlock();
	}
	
	explicit operator bool() const {
		return own;
	}

private:
	bool own;
	t    &core;
};


#endif //PM1_SDK_WEAK_LOCK_GUARD_HH
