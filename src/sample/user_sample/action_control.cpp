#include <iostream>
#include <iomanip>
#include <thread>
#include <conio.h>

#include "pm1_sdk.h"                 // ͷ�ļ�
using namespace autolabor::pm1;      // �����ռ�
using namespace std;

int main()
{
	cout << "initializing..." << endl;
	auto result = initialize();     // ��ʼ������
	if (result)
	{
		cout << "connected to " << result.value << "[PM1"
		     << (check_state() == chassis_state::unlocked ?
		         "����]" : "����/����]") << endl;
		cout << "����" << endl
			<< "[quit:		�˳�" << endl
			<< " lock:		����" << endl
			<< " unlock:	����" << endl
			<< " front:		ֱ��1m" << endl
			<< " back:		����1m" << endl
			<< " left:		��ת90��" << endl
			<< " left-:		�����90��" << endl
			<< " right:		��ת90��" << endl
			<< " right-:	�Һ���90��" << endl
			<< " inverse:	��ʱ��ת90��" << endl
			<< " clockwise:	˳ʱ��ת90��]" << endl;
		string cmd = "";
		while (true)
		{
			cin >> cmd;
			if (cmd == "quit")			// �˳�
			{
				break;
			}
			else if (cmd == "lock")		// ����
			{
				lock();
				while (check_state() != chassis_state::locked)
				{
					delay(0.1);
				}
				cout << "[������]" << endl;
			}
			else if (cmd == "unlock")	// ����
			{
				unlock();
				while (check_state() != chassis_state::unlocked)
				{
					delay(0.1);
				}
				cout << "[�ѽ���]" << endl;
			}
			else
			{
				double progress = 0;
				thread t;
				if (cmd == "front")			// ֱ��1m
				{
					t = thread([&] { go_straight(0.1, 1, &progress); });
				}
				else if (cmd == "back")		// ����1m
				{
					t = thread([&] { go_straight(-0.1, 1, &progress); });
				}
				else if (cmd == "left")		// ��ת90��
				{
					t = thread([&] { go_arc_va(0.1, 0.5, 3.14 / 2, &progress); });
				}
				else if (cmd == "left-")	// �����90��
				{
					t = thread([&] { go_arc_va(-0.1, 0.5, 3.14 / 2, &progress); });
				}
				else if (cmd == "right")	// ��ת90��
				{
					t = thread([&] { go_arc_va(0.1, -0.5, 3.14 / 2, &progress); });
				}
				else if (cmd == "right-")	// �Һ���90��
				{
					t = thread([&] { go_arc_va(-0.1, -0.5, 3.14 / 2, &progress); });
				}
				else if (cmd == "inverse")	// ��ʱ��ת90��
				{
					t = thread([&] { turn_around(0.25, 3.14 / 2, &progress); });
				}
				else if (cmd == "clockwise")// ˳ʱ��ת90��
				{
					t = thread([&] { turn_around(-0.25, 3.14 / 2, &progress); });
				}
				else
				{
					cout << "[unknown command]" << endl;
					continue;
				}
				cout << "[����Esc������ֹ����ִ�еĶ���]" << endl;
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
					if (_kbhit() && _getch() == 27) // Esc��
					{
						cancel_action();
						cout << "��������ֹ";
						break;
					}
				}
				t.join();
				cout << endl;
			}
		}
		shutdown();		// �Ͽ�����
	}
	else
	{
		cerr << "[" << result.error_info << "]" << endl;
		system("pause");
	}
	return 0;
}
