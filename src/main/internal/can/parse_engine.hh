//
// Created by ydrml on 2019/3/7.
//

#ifndef PM1_SDK_PARSE_ENGINE_HH
#define PM1_SDK_PARSE_ENGINE_HH


#include <deque>
#include <functional>
#include "parser.hh"

namespace autolabor {
	namespace can {
		/**
		 * 解析引擎
		 */
		class parse_engine {
			using callback_t = std::function<void(const parser::result &)>;
			
			std::deque<uint8_t> buffer;
			parser              parser;
			const callback_t    callback;
			
			unsigned int ptr;
			
			/** 进行一次尽力而为的解析 */
			void parse();
			
			/** 重新找到帧头 */
			void find_head(int);
		
		public:
			/** 置入回调 */
			explicit parse_engine(callback_t &&);
			
			/** 平凡复制 */
			parse_engine(const parse_engine &) = default;
			
			/** 禁止移动 */
			parse_engine(parse_engine &&) = delete;
			
			/** 进行解析 */
			void operator()(uint8_t);
		};
	}
}


#endif //PM1_SDK_PARSE_ENGINE_HH