#include <iostream>
#include "pm1_sdk.h"                 // 头文件

using namespace autolabor::pm1;      // 命名空间
using namespace std;

int main()
{
    cout << "initializing..." << endl;
    auto result = initialize();     // 初始化连接
	if (result)
	{
        cout << "connected to " << result.value << endl;
        unlock();                   // 解锁
		while (check_state() != chassis_state::unlocked)
		{
			delay(0.1);
		}
        cout << "moving..." << endl;
        turn_around(0.25, 1.57);    // 以0.25rad/s的速度原地转90°
        shutdown();                 // 断开连接
	}
	else
	{
        cerr << result.error_info << endl;
	}
	system("pause");
	return 0;
}
