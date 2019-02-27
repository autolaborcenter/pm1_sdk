//
// Created by ydrml on 2019/2/26.
//

#ifndef PM1_SDK_CAN_DEFINE_H
#define PM1_SDK_CAN_DEFINE_H

#include "can_message.h"

#include <array>

namespace {
	using sgn = autolabor::pm1::union_no_data;   // 信号，无数据
	using msg = autolabor::pm1::union_with_data; // 消息，有数据
}

namespace autolabor {
	namespace pm1 {
		constexpr uint8_t any_type  = 0x3f,
		                  any_index = 0x0f;
		
		/**
		 * CAN 包信息定义
		 *
		 * @tparam _data_t     使用的数据结构
		 * @tparam _network    网络号
		 * @tparam _property   优先级
		 * @tparam _node_type  节点类型
		 * @tparam _node_index 节点序号
		 * @tparam _type       消息类型
		 */
		template<class _data_t,
				uint8_t _network,
				uint8_t _property,
				uint8_t _node_type,
				uint8_t _node_index,
				uint8_t _type>
		class can_pack_info {
		public:
			using data_t = _data_t;
			
			static_assert(std::is_same<_data_t, msg>::value
			              || std::is_same<_data_t, sgn>::value,
			              "struct must be a can pack type");
			static_assert(_network <= mask(2), "network in 2 bits");
			static_assert(_property <= mask(3), "property in 3 bits");
			static_assert(_node_type <= mask(6), "node type in 6 bits");
			static_assert(_node_index <= mask(4), "node index in 4 bits");
			
			constexpr static auto network    = _network;
			constexpr static auto data_field = std::is_same<_data_t, msg>::value;
			constexpr static auto property   = _property;
			constexpr static auto node_type  = _node_type;
			constexpr static auto node_index = _node_index;
			constexpr static auto type       = _type;
			
			constexpr static uint8_t bytes[]{
					static_cast<uint8_t>((network << 6u)
					                     | (data_field ? (1u << 5u) : 0)
					                     | (property << 2u)
					                     | (node_type >> 4u)),
					static_cast<uint8_t>((node_type << 4u) | node_index),
					type};
			
			inline static bool match(const data_t &msg) {
				const auto info = msg.data.info();
				return info.node_type() == node_type
				       && info.node_index() == node_index
				       && msg.data.type == type;
			}
		};
		
		/** 动力控制器包信息协议 */
		template<uint8_t node_index = any_index>
		class ecu {
		public:
			constexpr static uint8_t type_id = 0x11;
			// 目标速度
			using target_speed        = can_pack_info<msg, 0, 0, type_id, node_index, 0x1>;
			// 当前速度
			using current_speed_tx    = can_pack_info<sgn, 0, 0, type_id, node_index, 0x5>;
			using current_speed_rx    = can_pack_info<msg, 0, 0, type_id, node_index, 0x5>;
			// 当前编码器读数
			// FIXME 去掉这一行将导致编译错误
			using current_position_tx = can_pack_info<sgn, 0, 0, type_id, node_index, 0x6>;
			using current_position_rx = can_pack_info<msg, 0, 0, type_id, node_index, 0x6>;
			// 超时时间
			using timeout             = can_pack_info<msg, 0, 0, type_id, node_index, 0xa>;
		};
		
		/** 动力控制器包信息协议 */
		template<uint8_t node_index = any_index>
		class tcu {
		public:
			constexpr static uint8_t type_id = 0x12;
			// 目标角度
			using target_position     = can_pack_info<msg, 0, 0, type_id, node_index, 0x1>;
			// 当前角度
			using current_position_tx = can_pack_info<sgn, 0, 0, type_id, node_index, 0x3>;
			using current_position_rx = can_pack_info<msg, 0, 0, type_id, node_index, 0x3>;
			// 当前速度
			using current_speed_tx    = can_pack_info<sgn, 0, 0, type_id, node_index, 0x5>;
			using current_speed_rx    = can_pack_info<msg, 0, 0, type_id, node_index, 0x5>;
		};
		
		/** 打包（无数据域） */
		template<class info_t>
		inline typename info_t::data_t pack(uint8_t reserve = 0) {
			using type = typename info_t::data_t;
			static_assert(std::is_same<type, sgn>::value, "cannot build a signal pack with message info");
			
			type msg{};
			std::memcpy(msg.bytes + 1, info_t::bytes, 3);
			msg.data.reserve = reserve;
			reformat(msg);
			return msg;
		}
		
		/** 打包（有数据域） */
		template<class info_t>
		inline typename info_t::data_t pack(std::array<uint8_t, 8> &&data, uint8_t frame_id = 0) {
			using type = typename info_t::data_t;
			static_assert(std::is_same<type, msg>::value, "cannot build a message pack with signal info");
			
			type msg{};
			std::memcpy(msg.bytes + 1, info_t::bytes, 3);
			std::memcpy(msg.data.data, data.data(), 8);
			msg.data.frame_id = frame_id;
			reformat(msg);
			return msg;
		}
	}
}

#endif //PM1_SDK_CAN_DEFINE_H
