# PM1 SDK

## 介绍

本工程是 Autolabor PM1 底盘简单、跨平台、无依赖的驱动库。使用 **C++11** 实现，提供一系列顶层函数，用于：
* 管理与机器人底盘的串口连接 
  * 列出接入操作系统的串口的名字
  * 与特定串口建立连接
  * 与自动选择的串口建立连接
  * 断开连接
* 操纵机器人底盘行驶 
  * 基于动作，阻塞地控制
  * 基于指令，连续地控制

## 开始使用

要使用本工程，你可以：

* 拉取源码，并使用 CMake 编译；

* 从[发布页](https://github.com/autolaborcenter/pm1_sdk/releases)直接下载库文件；

* 或从[这里](https://github.com/autolaborcenter/pm1_driver_ros)下载 ROS 驱动；

## 试一下！

你买的 PM1 底盘已经到货了吗？现在连接到计算机，来试一下吧！

```c++
#include "pm1_sdk.h"
using namespace autolabor::pm1;

int main() {
    initialize();
    turn_around(0.5, 1.57);
}
```

## 细节

要了解关于使用的更多细节，请浏览 [`wiki`](https://github.com/autolaborcenter/pm1_sdk/wiki/Home)
