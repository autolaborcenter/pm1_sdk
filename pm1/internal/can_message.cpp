//
// Created by ydrml on 2019/2/25.
//

#include "can_message.h"

#include <vector>

union union_info {
	uint8_t  bytes[2];
	uint16_t data;
};

/** 计算某个长度的遮盖 */
constexpr uint8_t musk(uint8_t length) { return 0xffu >> (8u - length); }

/**
 * 从数据中截取
 * @tparam p 从前数，第一位的位置
 * @tparam l 位数
 * @param data 数据
 * @return 片段
 */
template<uint8_t p, uint8_t l>
inline uint8_t get(uint16_t data) { return static_cast<uint8_t>(data >> (16u - p - l)) & musk(l); }

autolabor::pm1::can_message_info::can_message_info(uint8_t b0, uint8_t b1) : data(union_info{b1, b0}.data) {}

uint8_t autolabor::pm1::can_message_info::network() const { return get<0, 2>(data); }

bool autolabor::pm1::can_message_info::data_field() const { return get<2, 1>(data); }

uint8_t autolabor::pm1::can_message_info::property() const { return get<3, 3>(data); }

uint8_t autolabor::pm1::can_message_info::node_type() const { return get<6, 6>(data); }

uint8_t autolabor::pm1::can_message_info::node_index() const { return data & 0b1111u; }

autolabor::pm1::can_message_info autolabor::pm1::can_pack_no_data::info() const { return {info0, info1}; }

autolabor::pm1::can_message_info autolabor::pm1::can_pack_with_data::info() const { return {info0, info1}; }
