#include <iostream>
#include <iomanip>
#include <thread>
#include <conio.h>

#include "pm1_sdk.h"                 // 头文件
using namespace autolabor::pm1;      // 命名空间
using namespace std;

#ifdef _MSC_VER
#ifdef DEBUG
#pragma comment(lib, "pm1_sdk_debug.lib")
#else
#pragma comment(lib, "pm1_sdk.lib")
#endif
#endif

int main()
{
	cout << "initializing..." << endl;
	auto result = initialize();     // 初始化连接
	if (result)
	{
		cout << "connected to " << result.value << "[PM1"
		     << (check_state() == chassis_state::unlocked ?
		         "解锁]" : "锁定/错误]") << endl;
		cout << "动作" << endl
			<< "[quit:		退出" << endl
			<< " lock:		锁定" << endl
			<< " unlock:	解锁" << endl
			<< " front:		直行1m" << endl
			<< " back:		后退1m" << endl
			<< " left:		左转90°" << endl
			<< " left-:		左后退90°" << endl
			<< " right:		右转90°" << endl
			<< " right-:	右后退90°" << endl
			<< " inverse:	逆时针转90°" << endl
			<< " clockwise:	顺时针转90°]" << endl;
		string cmd = "";
		while (true)
		{
			cin >> cmd;
			if (cmd == "quit")			// 退出
			{
				break;
			}
			else if (cmd == "lock")		// 锁定
			{
				lock();
				while (check_state() != chassis_state::locked)
				{
					delay(0.1);
				}
				cout << "[已锁定]" << endl;
			}
			else if (cmd == "unlock")	// 解锁
			{
				unlock();
				while (check_state() != chassis_state::unlocked)
				{
					delay(0.1);
				}
				cout << "[已解锁]" << endl;
			}
			else
			{
				double progress = 0;
				thread t;
				if (cmd == "front")			// 直行1m
				{
					t = thread([&] { go_straight(0.1, 1, &progress); });
				}
				else if (cmd == "back")		// 后退1m
				{
					t = thread([&] { go_straight(-0.1, 1, &progress); });
				}
				else if (cmd == "left")		// 左转90°
				{
					t = thread([&] { go_arc_va(0.1, 0.5, 3.14 / 2, &progress); });
				}
				else if (cmd == "left-")	// 左后退90°
				{
					t = thread([&] { go_arc_va(-0.1, 0.5, 3.14 / 2, &progress); });
				}
				else if (cmd == "right")	// 右转90°
				{
					t = thread([&] { go_arc_va(0.1, -0.5, 3.14 / 2, &progress); });
				}
				else if (cmd == "right-")	// 右后退90°
				{
					t = thread([&] { go_arc_va(-0.1, -0.5, 3.14 / 2, &progress); });
				}
				else if (cmd == "inverse")	// 逆时针转90°
				{
					t = thread([&] { turn_around(0.25, 3.14 / 2, &progress); });
				}
				else if (cmd == "clockwise")// 顺时针转90°
				{
					t = thread([&] { turn_around(-0.25, 3.14 / 2, &progress); });
				}
				else
				{
					cout << "[unknown command]" << endl;
					continue;
				}
				cout << "[按下Esc键可终止正在执行的动作]" << endl;
				cout << "[ 0%]";
				double last = 0;
				while (progress < 1)
				{
					delay(0.1);
					cout << "\b\b\b\b";
					if ((progress - last) >= 0.01)
					{
						for (int i = 0; i < int((progress - last) / 0.01); i++)
						{
							cout << "=";
						}
						last = progress;
					}
					cout << setw(2) << int(progress * 100) << "%]";
					if (_kbhit() && _getch() == 27) // Esc键
					{
						cancel_action();
						cout << "动作已终止";
						break;
					}
				}
				t.join();
				cout << endl;
			}
		}
		shutdown();		// 断开连接
	}
	else
	{
		cerr << "[" << result.error_info << "]" << endl;
		system("pause");
	}
	return 0;
}
