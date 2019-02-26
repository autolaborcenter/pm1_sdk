//
// Created by ydrml on 2019/2/26.
//

#include "test.h"

#include "../main/pm1/api.h"
#include "../main/pm1/extensions.h"
#include "../main/pm1/time_extensions.h"
#include "../main/pm1/internal/serial/serial.h"
#include "../main/pm1/internal/can_message.h"
#include "../main/pm1/internal/can_define.h"
#include "../main/pm1/internal/parser.hh"

using namespace mechdancer::common;
using namespace autolabor::pm1;

void test::test_string_print() {
	println(join_to_string("", 1, 2, 3, 4, 5));
	println(join_to_string(", ", 1, 2, 3, 4, 5));
	println(join_to_string("", '[', join_to_string(", ", 1, 2, 3, 4, 5), ']'));
	
	println(measure_time([] { delay(1); }).count());
}

void test::test_serial_port() {
	try {
		//enumerate_ports
		for (const auto &it : serial::list_ports()) {
			std::cout << "("
			          << it.port.c_str() << ", "
			          << it.description.c_str() << ", "
			          << it.hardware_id.c_str() << ")"
			          << std::endl;
		}
		// port, baudrate, timeout in milliseconds
		serial::Serial  my_serial("com3", 9600, serial::Timeout::simpleTimeout(1000));
		
		std::cout << "Is the serial port open?"
		          << (my_serial.isOpen() ? " Yes." : " No.")
		          << std::endl;
		
		// Get the Test string
		int         count       = 0;
		std::string test_string = "Testing.";
		
		// Test the timeout, there should be 1 second between prints
		std::cout << "Timeout == 1000ms, asking for 1 more byte than written." << std::endl;
		while (count < 10) {
			size_t bytes_wrote = my_serial.write(test_string);
			
			std::string result = my_serial.read(test_string.length() + 1);
			
			std::cout << "Iteration: " << count
			          << ", Bytes written: " << bytes_wrote
			          << ", Bytes read: " << result.length()
			          << ", String read: " << result
			          << std::endl;
			
			count += 1;
		}
		
		// Test the timeout at 250ms
		my_serial.setTimeout(serial::Timeout::max(), 250, 0, 250, 0);
		count = 0;
		std::cout << "Timeout == 250ms, asking for 1 more byte than written." << std::endl;
		while (count < 10) {
			size_t bytes_wrote = my_serial.write(test_string);
			
			std::string result = my_serial.read(test_string.length() + 1);
			
			std::cout << "Iteration: " << count
			          << ", Bytes written: " << bytes_wrote
			          << ", Bytes read: " << result.length()
			          << ", String read: " << result
			          << std::endl;
			
			count += 1;
		}
		
		// Test the timeout at 250ms, but asking exactly for what was written
		count = 0;
		std::cout << "Timeout == 250ms, asking for exactly what was written." << std::endl;
		while (count < 10) {
			size_t bytes_wrote = my_serial.write(test_string);
			
			std::string result = my_serial.read(test_string.length());
			
			std::cout << "Iteration: " << count
			          << ", Bytes written: " << bytes_wrote
			          << ", Bytes read: " << result.length()
			          << ", String read: " << result
			          << std::endl;
			
			count += 1;
		}
		
		// Test the timeout at 250ms, but asking for 1 less than what was written
		count = 0;
		std::cout << "Timeout == 250ms, asking for 1 less than was written." << std::endl;
		while (count < 10) {
			size_t bytes_wrote = my_serial.write(test_string);
			
			std::string result = my_serial.read(test_string.length() - 1);
			
			std::cout << "Iteration: " << count
			          << ", Bytes written: " << bytes_wrote
			          << ", Bytes read: " << result.length()
			          << ", String read: " << result
			          << std::endl;
			
			count += 1;
		}
	} catch (std::exception &e) {
		std::cerr << "Unhandled Exception: " << e.what() << std::endl;
	}
}

void test::test_crc_check() {
	std::cout << sizeof(can_pack_no_data) << std::endl;
	std::cout << sizeof(can_pack_with_data) << std::endl;
	
	union_no_data
			temp1{0xfe, 0x31, 0x32, 0x33, 0x34, 0xf1};
	
	std::cout << std::boolalpha
	          << crc_check(temp1) << std::endl;
	
	union_no_data
			temp2{0x00, 0x31, 0x32, 0x33, 0x34, 0x00};
	
	reformat(temp2);
	
	std::cout << std::boolalpha
	          << (0xfe == temp2.bytes[0]) << std::endl
	          << (0xf1 == temp2.bytes[sizeof(temp2) - 1]) << std::endl;
}

template<class t>
class choose {};

template<>
class choose<can_pack_no_data> {
public:
	using union_t = union_no_data;
};

template<>
class choose<can_pack_with_data> {
public:
	using union_t = union_with_data;
};

template<class t, class union_t = typename choose<t>::union_t>
void display(t msg) {
	auto info = msg.info();
	std::cout << std::hex
	          << "network:\t0x" << (int) info.network() << std::endl
	          << std::boolalpha
	          << "data_field:\t" << info.data_field() << std::endl
	          << std::dec
	          << "property:\t" << (int) info.property() << std::endl
	          << std::hex
	          << "node_type:\t0x" << (int) info.node_type() << std::endl
	          << std::dec
	          << "node_index:\t" << (int) info.node_index() << std::endl
	          << std::hex
	          << "msg_type:\t0x" << (int) msg.type << std::endl;
	union_t temp{};
	temp.data = msg;
	std::cout << "crc_check:\t" << crc_check(temp) << std::endl
	          << "can pack:\t[ ";
	for (int b:temp.bytes) {
		if (b < 0x10)std::cout << '0';
		std::cout << b << " ";
	}
	std::cout << "]" << std::endl;
}

void test::test_pack() {
	display(pack<ecu<0>::current_speed_tx>());
	display(pack<ecu<1>::target_speed>({1, 2, 3, 4, 5, 6, 7, 8}));
}

void test::test_parse() {
	auto msg = pack<tcu<>::current_position_tx>();
	
	choose<decltype(msg)>::union_t space{};
	space.data = msg;
	
	parser    parser;
	for (auto b : space.bytes) {
		auto temp = parser(b);
		switch (temp.type) {
			case parser::result_type::nothing:
				break;
			case parser::result_type::signal:
				std::cout << "received: -------------------" << std::endl;
				display(temp.signal);
				break;
			case parser::result_type::message:
				std::cout << "received: -------------------" << std::endl;
				display(temp.message);
				break;
		}
	}
}
