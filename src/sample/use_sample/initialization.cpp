#include <iostream>
#include "pm1_sdk.h"                 // 头文件
using namespace autolabor::pm1;      // 命名空间

#ifdef _DEBUG                        // 静态库
#pragma comment(lib, "pm1_sdk_debug.lib")
#else
#pragma comment(lib, "pm1_sdk.lib")
#endif

int main()
{
	std::cout << "initializing..." << std::endl;
	auto result = initialize();     // 初始化连接
	if (result)
	{
		std::cout << "connected to " << result.value << std::endl;
		unlock();                   // 解锁
		while (check_state() != chassis_state::unlocked)
		{
			delay(0.1);
		}
		std::cout << "moving..." << std::endl;
		turn_around(0.25, 1.57);    // 以0.25rad/s的速度原地转90°
		shutdown();                 // 断开连接
	}
	else
	{
		std::cerr << result.error_info << std::endl;
	}
	system("pause");
	return 0;
}