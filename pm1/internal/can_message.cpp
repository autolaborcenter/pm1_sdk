//
// Created by ydrml on 2019/2/25.
//

#include <vector>
#include "can_message.h"

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

bool _crc_check(std::vector<uint8_t> &&bytes, uint8_t sum) {
	return false;
}

autolabor::pm1::can_message_info::can_message_info(uint8_t b0, uint8_t b1) : data({b1, b0}) {}

uint8_t autolabor::pm1::can_message_info::network() const { return get<0, 2>(data.data); }

bool autolabor::pm1::can_message_info::data_field() const { return get<2, 1>(data.data); }

uint8_t autolabor::pm1::can_message_info::property() const { return get<3, 3>(data.data); }

uint8_t autolabor::pm1::can_message_info::node_type() const { return get<6, 6>(data.data); }

uint8_t autolabor::pm1::can_message_info::node_index() const { return data.data & 0b1111u; }

std::array<uint8_t, 2> autolabor::pm1::can_message_info::bytes() const {
	return {data.bytes[1], data.bytes[0]};
}

bool autolabor::pm1::can_message::crc_check(uint8_t sum) {
	auto temp = head.info.bytes();
	return _crc_check({head.head, temp[0], temp[1], head.msg_type,
	                   reserved
	                  }, sum);
}

bool autolabor::pm1::can_message_with_data::crc_check(uint8_t sum) {
	auto temp = head.info.bytes();
	return _crc_check({head.head, temp[0], temp[1], head.msg_type,
	                   frame_id,
	                   data[0], data[1], data[2], data[3],
	                   data[4], data[5], data[6], data[7],
	                  }, sum);
}
