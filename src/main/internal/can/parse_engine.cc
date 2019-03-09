//
// Created by ydrml on 2019/3/7.
//

#include "parse_engine.hh"

using namespace autolabor::can;

parse_engine::parse_engine(parse_engine::callback_t &&callback)
		: callback(std::move(callback)), ptr(0) {}

void parse_engine::operator()(uint8_t byte) {
	buffer.push_back(byte);
	parse();
}

void parse_engine::parse() {
	while (ptr < buffer.size()) {
		auto result = parser(buffer[ptr++]);
		switch (result.type) {
			case parser::result_type::nothing:
				break;
			case parser::result_type::failed:
			case parser::result_type::signal_failed:
			case parser::result_type::message_failed:
				find_head(1);
				break;
			case parser::result_type::signal:
				callback(result);
				find_head(sizeof(pack_no_data));
				break;
			case parser::result_type::message:
				callback(result);
				find_head(sizeof(pack_with_data));
				break;
		}
	}
}

void parse_engine::find_head(int n) {
	while (n-- > 0)
		buffer.pop_front();
	while (!buffer.empty() && buffer.front() != 0xfe)
		buffer.pop_front();
	ptr = 0;
}
