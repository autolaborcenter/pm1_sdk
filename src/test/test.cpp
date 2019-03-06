//
// Created by ydrml on 2019/2/26.
//

#include "gtest/gtest.h"
#include "../main/internal/mechanical.h"

using namespace autolabor::pm1;

TEST(mechanical_state, straight) { // NOLINT(cert-err58-cpp)
	mechanical::state state(1, 0);
	
	ASSERT_DOUBLE_EQ(state.rho, 1);
	ASSERT_DOUBLE_EQ(state.rudder, 0);
	
	ASSERT_TRUE(std::isinf(state.r));
	ASSERT_NEAR(state.polar, -mechanical::pi / 2, 1E-6);
	
	ASSERT_NEAR(state.v_rate, 1, 1E-6);
	ASSERT_NEAR(state.w_rate, 0, 1E-6);
	
	ASSERT_NEAR(state.v, mechanical::max_v, 1E-6);
	ASSERT_NEAR(state.w, 0, 1E-6);
	
	ASSERT_NEAR(state.left, mechanical::max_v, 1E-6);
	ASSERT_NEAR(state.right, mechanical::max_v, 1E-6);
}

TEST(mechanical_state, turn) { // NOLINT(cert-err58-cpp)
	mechanical::state state(1, mechanical::pi / 2);
	
	ASSERT_DOUBLE_EQ(state.rho, 1);
	ASSERT_DOUBLE_EQ(state.rudder, mechanical::pi / 2);
	
	ASSERT_NEAR(state.r, 0, 1E-6);
	ASSERT_NEAR(state.polar, 0, 1E-6);
	
	ASSERT_NEAR(state.v_rate, 0, 1E-6);
	ASSERT_NEAR(state.w_rate, -1, 1E-6);
	
	ASSERT_NEAR(state.v, 0, 1E-6);
	ASSERT_NEAR(state.w, -mechanical::max_w, 1E-6);
	
	ASSERT_NEAR(state.left, +mechanical::max_v, 1E-6);
	ASSERT_NEAR(state.right, -mechanical::max_v, 1E-6);
}

TEST(mechanical_state, arc) { // NOLINT(cert-err58-cpp)
	mechanical::state state(1, -mechanical::pi / 4);
	
	ASSERT_DOUBLE_EQ(state.rho, 1);
	ASSERT_DOUBLE_EQ(state.rudder, -mechanical::pi / 4);
	
	ASSERT_TRUE(state.r > 0);
	ASSERT_TRUE(state.polar > 0);
	
	ASSERT_TRUE(state.v_rate > 0);
	ASSERT_TRUE(state.w_rate > 0);
	
	ASSERT_TRUE(state.left > 0);
	ASSERT_TRUE(state.right > 0);
	ASSERT_TRUE(state.left < state.right);
}

TEST(mechanical_state, build_straight) { // NOLINT(cert-err58-cpp)
	auto state = mechanical::state::from_target(mechanical::max_v, 0);
	
	ASSERT_NEAR(state->rho, 1, 1E-6);
	ASSERT_NEAR(state->rudder, 0, 1E-6);
	
	ASSERT_TRUE(std::isinf(state->r));
	ASSERT_NEAR(state->polar, -mechanical::pi / 2, 1E-6);
	
	ASSERT_NEAR(state->v_rate, 1, 1E-6);
	ASSERT_NEAR(state->w_rate, 0, 1E-6);
	
	ASSERT_NEAR(state->v, mechanical::max_v, 1E-6);
	ASSERT_NEAR(state->w, 0, 1E-6);
	
	ASSERT_NEAR(state->left, mechanical::max_v, 1E-6);
	ASSERT_NEAR(state->right, mechanical::max_v, 1E-6);
}

TEST(mechanical_state, build_turn) { // NOLINT(cert-err58-cpp)
	auto state = mechanical::state::from_target(0, mechanical::max_w);
	
	ASSERT_NEAR(state->rho, 1, 1E-6);
	ASSERT_NEAR(state->rudder, -mechanical::pi / 2, 1E-6);
	
	ASSERT_NEAR(state->r, 0, 1E-6);
	ASSERT_NEAR(state->polar, 0, 1E-6);
	
	ASSERT_NEAR(state->v_rate, 0, 1E-6);
	ASSERT_NEAR(state->w_rate, 1, 1E-6);
	
	ASSERT_NEAR(state->v, 0, 1E-6);
	ASSERT_NEAR(state->w, mechanical::max_w, 1E-6);
	
	ASSERT_NEAR(state->left, -mechanical::max_v, 1E-6);
	ASSERT_NEAR(state->right, +mechanical::max_v, 1E-6);
}
