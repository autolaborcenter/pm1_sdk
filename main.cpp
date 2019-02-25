//
// Created by ydrml on 2019/2/22.
//

void test_string_print();

void test_serial_port();

#include <iostream>
#include "pm1/api.h"
#include "pm1/internal/serial/serial.h"

#include "pm1/extensions.h"
#include "pm1/time_extensions.h"
#include "pm1/internal/chassis.hh"
#include "pm1/internal/can_message.h"

using namespace mechdancer::common;

int main() {
	//	auto temp = autolabor::pm1::can_message_info(104, 52);
	//	std::cout << (int) temp.network() << std::endl
	//	          << std::boolalpha << temp.data_field() << std::endl
	//	          << (int) temp.property() << std::endl
	//	          << (int) temp.node_type() << std::endl
	//	          << (int) temp.node_index() << std::endl
	//	          << (int) temp.bytes()[0] << std::endl
	//	          << (int) temp.bytes()[1] << std::endl;
	
	std::cout << sizeof(autolabor::pm1::can_pack_no_data) << std::endl;
	std::cout << sizeof(autolabor::pm1::can_pack_with_data) << std::endl;
	
	autolabor::pm1::union_no_data temp{};
	temp.bytes[0] = 100;
	std::cout << (int) temp.data.head << std::endl;
	std::cout << (int) temp.data.crc << std::endl;
}

void test_string_print() {
	println(join_to_string("", 1, 2, 3, 4, 5));
	println(join_to_string(", ", 1, 2, 3, 4, 5));
	println(join_to_string("", '[', join_to_string(", ", 1, 2, 3, 4, 5), ']'));
	
	println(measure_time([] { autolabor::pm1::delay(1); }).count());
}

void test_serial_port() {
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
