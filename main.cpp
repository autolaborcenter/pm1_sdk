//
// Created by ydrml on 2019/2/22.
//

#include <thread>
#include <iostream>
#include "pm1/extensions.h"
#include "pm1/time_extensions.h"
#include "pm1/api.h"
#include "pm1/internal/chassis.h"

#include <string>
#include <iostream>
#include <cstdio>

// OS Specific sleep
#ifdef _WIN32

#include <windows.h>

#else
#include <unistd.h>
#endif

#include "pm1/internal/serial/serial.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

using namespace mechdancer::common;

template<class _, class __>
inline void println(std::chrono::duration<_, __> duration) {
	println(duration.count());
}

//int main() {
//	println(join_to_string("", 1, 2, 3, 4, 5));
//	println(join_to_string(", ", 1, 2, 3, 4, 5));
//	println(join_to_string("", '[', join_to_string(", ", 1, 2, 3, 4, 5), ']'));
//
//	println(measure_time([] { autolabor::pm1::delay(1); }));
//
//	const auto chassis = autolabor::pm1::chassis::instance();
//	std::cout << chassis << std::endl;
//
//	return 0;
//}

void enumerate_ports() {
	vector<serial::PortInfo> devices_found = serial::list_ports();
	
	vector<serial::PortInfo>::iterator iter = devices_found.begin();
	
	while (iter != devices_found.end()) {
		serial::PortInfo device = *iter++;
		
		printf("(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
		       device.hardware_id.c_str());
	}
}

int main(int argc, char **argv) {
	try {
		enumerate_ports();
		// port, baudrate, timeout in milliseconds
		serial::Serial my_serial("com3", 9600, serial::Timeout::simpleTimeout(1000));
		
		cout << "Is the serial port open?";
		if (my_serial.isOpen())
			cout << " Yes." << endl;
		else
			cout << " No." << endl;
		
		// Get the Test string
		int    count = 0;
		string test_string;
		if (argc == 4) {
			test_string = argv[3];
		} else {
			test_string = "Testing.";
		}
		
		// Test the timeout, there should be 1 second between prints
		cout << "Timeout == 1000ms, asking for 1 more byte than written." << endl;
		while (count < 10) {
			size_t bytes_wrote = my_serial.write(test_string);
			
			string result = my_serial.read(test_string.length() + 1);
			
			cout << "Iteration: " << count << ", Bytes written: ";
			cout << bytes_wrote << ", Bytes read: ";
			cout << result.length() << ", String read: " << result << endl;
			
			count += 1;
		}
		
		// Test the timeout at 250ms
		my_serial.setTimeout(serial::Timeout::maxx(), 250, 0, 250, 0);
		count = 0;
		cout << "Timeout == 250ms, asking for 1 more byte than written." << endl;
		while (count < 10) {
			size_t bytes_wrote = my_serial.write(test_string);
			
			string result = my_serial.read(test_string.length() + 1);
			
			cout << "Iteration: " << count << ", Bytes written: ";
			cout << bytes_wrote << ", Bytes read: ";
			cout << result.length() << ", String read: " << result << endl;
			
			count += 1;
		}
		
		// Test the timeout at 250ms, but asking exactly for what was written
		count = 0;
		cout << "Timeout == 250ms, asking for exactly what was written." << endl;
		while (count < 10) {
			size_t bytes_wrote = my_serial.write(test_string);
			
			string result = my_serial.read(test_string.length());
			
			cout << "Iteration: " << count << ", Bytes written: ";
			cout << bytes_wrote << ", Bytes read: ";
			cout << result.length() << ", String read: " << result << endl;
			
			count += 1;
		}
		
		// Test the timeout at 250ms, but asking for 1 less than what was written
		count = 0;
		cout << "Timeout == 250ms, asking for 1 less than was written." << endl;
		while (count < 10) {
			size_t bytes_wrote = my_serial.write(test_string);
			
			string result = my_serial.read(test_string.length() - 1);
			
			cout << "Iteration: " << count << ", Bytes written: ";
			cout << bytes_wrote << ", Bytes read: ";
			cout << result.length() << ", String read: " << result << endl;
			
			count += 1;
		}
		
		return 0;
	} catch (exception &e) {
		cerr << "Unhandled Exception: " << e.what() << endl;
	}
}
