//
// Created by ydrml on 2019/2/25.
//

#ifndef PM1_SDK_CAN_MESSAGE_H
#define PM1_SDK_CAN_MESSAGE_H

#include <array>

namespace autolabor {
	namespace pm1 {
		union buffer16 {
			uint8_t  bytes[2];
			uint16_t data;
		};
		
		/** CAN 包信息 */
		struct can_message_info {
		public:
			/** 用两个字节构造 */
			can_message_info(uint8_t b0, uint8_t b1);
			
			/** 网络号 */
			uint8_t network() const;
			
			/** 有无数据域 */
			bool data_field() const;
			
			/** 优先级 */
			uint8_t property() const;
			
			/** 节点类型 */
			uint8_t node_type() const;
			
			/** 节点序号 */
			uint8_t node_index() const;
			
			/** 读出字节 */
			std::array<uint8_t, 2> bytes() const;
		
		private:
			/** 数据存储 */
			const buffer16 data;
		};
		
		/** CAN 包头 */
		struct can_message_head {
			uint8_t          head;
			can_message_info info;
			uint8_t          msg_type;
		};
		
		/** CAN 包（无数据域） */
		struct can_message {
			can_message_head head;
			uint8_t          reserved;
			
			/**
			 * 循环冗余校验
			 * @param sum 校验和
			 * @return 是否通过
			 */
			bool crc_check(uint8_t sum);
		};
		
		/** CAN 包（有数据域） */
		struct can_message_with_data {
			can_message_head       head;
			uint8_t                frame_id;
			std::array<uint8_t, 8> data;
			
			/**
			 * 循环冗余校验
			 * @param sum 校验和
			 * @return 是否通过
			 */
			bool crc_check(uint8_t sum);
		};
	}
}

#endif //PM1_SDK_CAN_MESSAGE_H
