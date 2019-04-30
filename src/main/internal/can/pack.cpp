//
// Created by ydrml on 2019/2/25.
//

#include "pack.h"

using namespace autolabor::can;

/**
 * 从数据中截取
 * @tparam p 从前数，第一位的位置
 * @tparam l 位数
 * @param data 数据
 * @return 片段
 */
template<uint8_t p, uint8_t l>
inline uint8_t get(uint16_t data) { return static_cast<uint8_t>(data >> (16u - p - l)) & mask(l); }

info_bytes::info_bytes(uint8_t b0, uint8_t b1) : data(msg_union<uint16_t>{b1, b0}.data) {}

uint8_t info_bytes::network() const { return get<0, 2>(data); }

bool info_bytes::data_field() const { return get<2, 1>(data); }

uint8_t info_bytes::property() const { return get<3, 3>(data); }

uint8_t info_bytes::node_type() const { return get<6, 6>(data); }

uint8_t info_bytes::node_index() const { return data & mask(4); }

info_bytes pack_no_data::info() const { return {info0, info1}; }

info_bytes pack_with_data::info() const { return {info0, info1}; }
