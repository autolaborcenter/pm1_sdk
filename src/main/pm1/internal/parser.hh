//
// Created by ydrml on 2019/2/26.
//

#ifndef PM1_SDK_PARSER_H
#define PM1_SDK_PARSER_H

#include "can_message.h"

namespace autolabor {
	namespace pm1 {
		class parser {
		public:
			enum class result_type : uint8_t {
				nothing, signal, message
			};
			
			struct result {
				result_type data_field;
				union {
					can_pack_no_data   signal;
					can_pack_with_data message;
				};
			};
			
			parser() = default;
			
			parser(const parser &others) = delete;
			
			parser(parser &&others) = delete;
			
			result parse(uint8_t byte);
		
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
			
			union_no_data   sgn_buffer{0xfe};
			union_with_data msg_buffer{0xfe};
			state_t         state{0};
		};
	}
}


#endif //PM1_SDK_PARSER_H
