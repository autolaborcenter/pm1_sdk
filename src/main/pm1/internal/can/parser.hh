//
// Created by ydrml on 2019/2/26.
//

#ifndef PM1_SDK_PARSER_H
#define PM1_SDK_PARSER_H

#include "can_message.h"

namespace autolabor {
	namespace pm1 {
		/** 解析器 */
		class parser {
		public:
			/** 结果类型 */
			enum class result_type : uint8_t {
				/** 无（不完整或校验失败） */
						nothing,
				/** 信号 */
						signal,
				/** 消息 */
						message
			};
			
			/** 结果结构体 */
			struct result {
				/** 结果类型 */
				result_type type;
				/** 二选一 */
				union {
					union_no_data   signal;
					union_with_data message;
				};
			};
			
			parser() = default;
			
			/** 禁止复制 */
			parser(const parser &others) = delete;
			
			/** 禁止移动 */
			parser(parser &&others) = delete;
			
			/** 逐字节解析 */
			result operator()(uint8_t byte);
		
		private:
			enum class state_type : uint8_t {
				origin,
				determine,
				signal,
				message,
				ending
			};
			
			struct state_t {
				uint8_t value;
				
				state_type state();
			};
			
			union {
				uint8_t bytes[sizeof(can_pack_with_data)];
				
				union_no_data   sgn_buffer;
				union_with_data msg_buffer;
			};
			state_t state{0};
		};
	}
}


#endif //PM1_SDK_PARSER_H
