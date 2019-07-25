#include <iostream>
#include <Windows.h>
#include "pm1_sdk.h"                 // 头文件

using namespace autolabor::pm1;      // 命名空间
using namespace std;

int main() {
    cout << "initializing..." << endl;
    auto result = initialize();    // 初始化连接
    if (result) {
        cout << "connected to " << result.value << endl;
        unlock();                // 解锁
        cout << "操作\n[方向键控制前后左右]\n[Esc键退出程序]" << endl;
        while (GetKeyState(VK_ESCAPE) >= 0) {
            double v     = 0, w = 0;
            bool   up    = GetKeyState(VK_UP) < 0;
            bool   down  = GetKeyState(VK_DOWN) < 0;
            bool   left  = GetKeyState(VK_LEFT) < 0;
            bool   right = GetKeyState(VK_RIGHT) < 0;
            if (up && !down && !left && !right)        // 前
            {
                v = 0.1;
                w = 0;
            } else if (up && !down && left && !right)    // 左前
            {
                v = 0.1;
                w = 0.2;
            } else if (up && !down && !left && right)    // 右前
            {
                v = 0.1;
                w = -0.2;
            } else if (!up && down && !left && !right)// 后
            {
                v = -0.1;
                w = 0;
            } else if (!up && down && left && !right)    // 左后
            {
                v = -0.1;
                w = -0.2;
            } else if (!up && down && !left && right)    // 右后
            {
                v = -0.1;
                w = 0.2;
            } else if (!up && !down && left && !right)// 逆时针原地转
            {
                v = 0;
                w = 0.2;
            } else if (!up && !down && !left && right)// 顺时针原地转
            {
                v = 0;
                w = -0.2;
            }
            drive(v, w);
            delay(0.1);
        }
        shutdown();        // 断开连接
    } else {
        cerr << "[" << result.error_info << "]" << endl;
        system("pause");
    }
    return 0;
}
