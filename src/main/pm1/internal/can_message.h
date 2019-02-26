//
// Created by ydrml on 2019/2/25.
//

#ifndef PM1_SDK_CAN_MESSAGE_H
#define PM1_SDK_CAN_MESSAGE_H

#include <array>

namespace {
	uint8_t CRC8Table[256] = {
			0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
			157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
			35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
			190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
			70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
			219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
			101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
			248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
			140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
			17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
			175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
			50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
			202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
			87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
			233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
			116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
	};
	
	/** 计算某个长度的遮盖 */
	constexpr uint8_t mask(uint8_t length) { return 0xffu >> (8u - length); }
	
	/**
	 * 转换器
	 * @tparam t 内容类型
	 */
	template<class t>
	union msg_union {
		uint8_t bytes[sizeof(t)];
		t       data;
	};
	
	/** 循环冗余计算 */
	template<class t, int last_index = sizeof(t) - 1>
	uint8_t crc_calculate(const msg_union<t> &msg) {
		uint8_t checksum = 0;
		
		for (auto i = 1; i < last_index; ++i)
			checksum = CRC8Table[checksum ^ msg.bytes[i]];
		
		return checksum;
	}
}

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
			uint16_t data;
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
		using union_no_data = msg_union<can_pack_no_data>;
		
		/** 有数据域 CAN 包转换器 */
		using union_with_data = msg_union<can_pack_with_data>;
		
		/** 循环冗余校验 */
		template<class t, int last_index = sizeof(t) - 1>
		bool crc_check(const msg_union<t> &msg) {
			return msg.bytes[last_index] == crc_calculate(msg);
		}
		
		/** 填充校验和 */
		template<class t, int last_index = sizeof(t) - 1>
		void reformat(msg_union<t> &msg) {
			msg.bytes[0]          = 0xfe;
			msg.bytes[last_index] = crc_calculate(msg);
		}
		
		
	}
}

#endif //PM1_SDK_CAN_MESSAGE_H
