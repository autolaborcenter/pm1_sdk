//
// Created by ydrml on 2019/2/25.
//

#ifndef PM1_SDK_CAN_MESSAGE_H
#define PM1_SDK_CAN_MESSAGE_H

#include <array>

namespace autolabor {
	namespace pm1 {
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
		
		private:
			/** 数据存储 */
			const uint16_t data;
		};
		
		/** 无数据域 CAN 包 */
		struct can_pack_no_data {
			uint8_t head;
			uint8_t info0;
			uint8_t info1;
			uint8_t type;
			uint8_t reserve;
			uint8_t crc;
			
			can_message_info info() const;
		};
		
		/** 有数据域 CAN 包 */
		struct can_pack_with_data {
			uint8_t head;
			uint8_t info0;
			uint8_t info1;
			uint8_t type;
			uint8_t frame_id;
			uint8_t data[8];
			uint8_t crc;
			
			can_message_info info() const;
		};
		
		/** 无数据域 CAN 包转换器 */
		union union_no_data {
			uint8_t          bytes[6];
			can_pack_no_data data;
		};
		
		/** 有数据域 CAN 包转换器 */
		union union_with_data {
			uint8_t            bytes[14];
			can_pack_with_data data;
		};
		
		/** 循环冗余校验 */
		template<uint8_t length>
		bool crc_check(const uint8_t *bytes) {
		
		}
	}
}

#endif //PM1_SDK_CAN_MESSAGE_H
