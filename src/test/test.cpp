//
// Created by ydrml on 2019/2/26.
//

#include "gtest/gtest.h"

extern "C" {
#include "../main/internal/control_model/model.h"
}

TEST(pm1_model, physical_to_wheels) { // NOLINT(cert-err58-cpp)
	{
		physical x{1, 0};
		auto     result = physical_to_wheels(&x, &default_config);
		ASSERT_EQ(result.left, default_config.max_wheel_speed);
		ASSERT_EQ(result.right, default_config.max_wheel_speed);
	}
}
