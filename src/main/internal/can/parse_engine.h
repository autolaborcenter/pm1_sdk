//
// Created by ydrml on 2019/3/7.
//

#ifndef PM1_SDK_PARSE_ENGINE_H
#define PM1_SDK_PARSE_ENGINE_H


#include <deque>
#include <functional>
#include "parser.hh"

namespace autolabor {
	namespace pm1 {
		class parse_engine {
			using callback_t = std::function<void(const parser::result &)>;
			
			std::deque<uint8_t> buffer;
			parser              parser;
			callback_t          callback;
			
			unsigned int ptr;
			
			void parse();
			
			void find_head(int);
		
		public:
			explicit parse_engine(callback_t &&callback);
			
			void operator()(uint8_t byte);
		};
	}
}


#endif //PM1_SDK_PARSE_ENGINE_H
