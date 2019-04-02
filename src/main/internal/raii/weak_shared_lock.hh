//
// Created by User on 2019/4/2.
//

#ifndef PM1_SDK_WEAK_SHARED_LOCK_HH
#define PM1_SDK_WEAK_SHARED_LOCK_HH


#include <shared_mutex>

class weak_shared_lock {
public:
	explicit weak_shared_lock(std::shared_mutex &);
	
	~weak_shared_lock();
	
	operator bool() const;

private:
	bool              own;
	std::shared_mutex &core;
};


#endif //PM1_SDK_WEAK_SHARED_LOCK_HH
