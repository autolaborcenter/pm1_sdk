//
// Created by ydrml on 2019/5/31.
//

#include "chassis_state_t.hh"

autolabor::pm1::chassis_state_t::iterator::iterator
    (autolabor::pm1::chassis_state_t &master, int i)
    : master(master), i(i) {}

autolabor::pm1::chassis_state_t::iterator &
autolabor::pm1::chassis_state_t::iterator::operator=
    (const autolabor::pm1::chassis_state_t::iterator &others) { return *this; }

bool
autolabor::pm1::chassis_state_t::iterator::operator!=
    (const autolabor::pm1::chassis_state_t::iterator &others) const { return i != others.i; }

autolabor::pm1::chassis_state_t::iterator &
autolabor::pm1::chassis_state_t::iterator::operator++() {
    ++i;
    return *this;
}

autolabor::pm1::node_state_t
autolabor::pm1::chassis_state_t::iterator::operator*() const { return master.states[i]; }

autolabor::pm1::chassis_state_t::iterator
autolabor::pm1::chassis_state_t::begin() { return iterator(*this, 0); }

autolabor::pm1::chassis_state_t::iterator
autolabor::pm1::chassis_state_t::end() { return iterator(*this, size); }
