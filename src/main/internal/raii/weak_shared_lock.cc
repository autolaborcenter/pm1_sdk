//
// Created by User on 2019/4/2.
//

#include "weak_shared_lock.hh"

weak_shared_lock::weak_shared_lock(std::shared_mutex &core)
		: core(core), own(core.try_lock_shared()) {}

weak_shared_lock::~weak_shared_lock() {
	if (own) core.unlock_shared();
}

weak_shared_lock::operator bool() const {
	return own;
}
