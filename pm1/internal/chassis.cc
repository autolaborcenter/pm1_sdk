//
// Created by ydrml on 2019/2/25.
//

#include "chassis.hh"

#include <mutex>

const auto my_timeout = serial::Timeout(serial::Timeout::max(), 1, 0, 0, 0); // NOLINT(cert-err58-cpp)

autolabor::pm1::chassis::chassis(const std::string &port_name)
		: port(new serial::Serial(port_name, 9600, my_timeout)) {
	// 导出 port 引用到符号
	auto port_ptr = this->port;
	// 启动接收线程
	std::thread([this, port_ptr] {
		std::string   buffer;
		unsigned char temp;
		while (port_ptr->isOpen()) {
			if (this->readByte(temp))
				buffer.push_back(temp);
			else if (port_ptr->isOpen())
				std::this_thread::sleep_for(std::chrono::milliseconds(50));
		}
		buffer.push_back(0);
		std::cout << buffer << std::endl;
	}).detach();
}

autolabor::pm1::chassis::~chassis() {
	port->close();
}

void autolabor::pm1::chassis::test_serial() {
	std::cout << "serial open: "
	          << std::boolalpha << port->isOpen()
	          << std::endl;
	
	for (int i = 0; i < 3; ++i) {
		port->write(mechdancer::common::join_to_string("", "test", i));
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
}

bool autolabor::pm1::chassis::readByte(unsigned char &byte) {
	// 临界资源
	static std::mutex  mutex;
	static std::string temp;
	
	// 内部同步
	std::lock_guard<std::mutex> lock(mutex);
	// 尝试读取
	try { temp = port->read(1); }
	catch (std::exception &e) { return false; }
	return temp.empty() ? false : (byte = temp[0], true);
}
