//
// Created by ydrml on 2019/2/26.
//

#ifndef PM1_SDK_CAN_DEFINE_H
#define PM1_SDK_CAN_DEFINE_H

#include "can/info.h"

#include <array>

namespace {
	using sgn = autolabor::can::union_no_data;   // 信号，无数据
	using msg = autolabor::can::union_with_data; // 消息，有数据
}

namespace autolabor {
	namespace pm1 {
		/**
		 * 广播地址常量
		 */
		constexpr uint8_t any_type  = 0x3f,
		                  any_index = 0x0f;
		
		/** 任意类型控制器结构 */
		struct any_controller {
			constexpr static uint8_t type_id    = any_type;
			constexpr static uint8_t node_index = any_index;
		};
		
		/** 通用控制器消息 */
		template<class type = any_controller>
		struct unit {
			constexpr static uint8_t type_id    = type::type_id;
			constexpr static uint8_t node_index = type::node_index;
			
			template<uint8_t msg_type_id>
			struct pack_info_pair {
				using tx = autolabor::can::info<sgn, 0, 0, type_id, node_index, msg_type_id>;
				using rx = autolabor::can::info<msg, 0, 0, type_id, node_index, msg_type_id>;
			};
			
			// 状态
			using state_tx       = typename pack_info_pair<0x80>::tx;
			using state_rx       = typename pack_info_pair<0x80>::rx;
			// 版本 id
			using version_id_tx  = typename pack_info_pair<0x81>::tx;
			using version_id_rx  = typename pack_info_pair<0x81>::rx;
			// 设备 id
			using device_id_tx   = typename pack_info_pair<0x82>::tx;
			using device_id_rx   = typename pack_info_pair<0x82>::rx;
			// 芯片 id
			using chip_id_tx     = typename pack_info_pair<0x83>::tx;
			using chip_id_rx     = typename pack_info_pair<0x83>::rx;
			// HAL 版本
			using hal_version_tx = typename pack_info_pair<0x84>::tx;
			using hal_version_rx = typename pack_info_pair<0x84>::rx;
			// 核心板硬件版本
			using core_hardware_version_tx  = typename pack_info_pair<0x85>::tx;
			using core_hardware_version_rx  = typename pack_info_pair<0x85>::rx;
			// 扩展板硬件版本
			using extra_hardware_version_tx = typename pack_info_pair<0x86>::tx;
			using extra_hardware_version_rx = typename pack_info_pair<0x86>::rx;
			// 软件版本
			using software_version_tx = typename pack_info_pair<0x87>::tx;
			using software_version_rx = typename pack_info_pair<0x87>::rx;
			// 累计运行时间
			using uptime_tx      = typename pack_info_pair<0x88>::tx;
			using uptime_rx      = typename pack_info_pair<0x88>::rx;
		};
		
		/** 动力控制器包信息协议 */
		template<uint8_t _node_index = any_index>
		class ecu {
		public:
			constexpr static uint8_t type_id    = 0x11;
			constexpr static uint8_t node_index = _node_index;
			// 目标速度
			using target_speed        = autolabor::can::info<msg, 0, 0, type_id, node_index, 0x1>;
			// 当前速度
			using current_speed_tx    = autolabor::can::info<sgn, 0, 0, type_id, node_index, 0x5>;
			using current_speed_rx    = autolabor::can::info<msg, 0, 0, type_id, node_index, 0x5>;
			// 当前编码器读数
			using current_position_tx = autolabor::can::info<sgn, 0, 0, type_id, node_index, 0x6>;
			using current_position_rx = autolabor::can::info<msg, 0, 0, type_id, node_index, 0x6>;
			// 编码器清零
			using clear               = autolabor::can::info<sgn, 0, 0, type_id, node_index, 0x7>;
			// 超时时间
			using timeout             = autolabor::can::info<msg, 0, 0, type_id, node_index, 0xa>;
		};
		
		/** 动力控制器包信息协议 */
		template<uint8_t _node_index = any_index>
		class tcu {
		public:
			constexpr static uint8_t type_id    = 0x12;
			constexpr static uint8_t node_index = _node_index;
			// 目标角度
			using target_position     = autolabor::can::info<msg, 0, 0, type_id, node_index, 0x1>;
			// 当前角度
			using current_position_tx = autolabor::can::info<sgn, 0, 0, type_id, node_index, 0x3>;
			using current_position_rx = autolabor::can::info<msg, 0, 0, type_id, node_index, 0x3>;
			// 当前速度
			using current_speed_tx    = autolabor::can::info<sgn, 0, 0, type_id, node_index, 0x5>;
			using current_speed_rx    = autolabor::can::info<msg, 0, 0, type_id, node_index, 0x5>;
		};
		
		/** 打包（无数据域） */
		template<class info_t>
		inline typename info_t::data_t pack(uint8_t reserve = 0) {
			using type = typename info_t::data_t;
			static_assert(std::is_same<type, sgn>::value, "cannot build a signal pack with message info");
			
			type msg{};
			std::memcpy(msg.bytes + 1, info_t::bytes, 3);
			msg.data.reserve = reserve;
			autolabor::can::reformat(msg);
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
			autolabor::can::reformat(msg);
			return msg;
		}
	}
}

#endif //PM1_SDK_CAN_DEFINE_H
