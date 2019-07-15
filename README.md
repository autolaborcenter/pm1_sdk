# PM1 SDK

## 介绍

原生应用是 Autolabor PM1 支持的首要开发方式。.Net 平台上的 SDK 也依赖于原生 SDK。

所谓*原生（Native）*，指的是不运行在任何虚拟机或程序框架中，而是编译到对应平台的机器码，仅依赖于操作系统即可执行的应用程序或程序库。典型的原生库常常是使用 C 或 C++ 编写，PM1 SDK 也是如此。

## 环境要求

编译 PM1 SDK 需要编译器实现 C++17 的一部分特性，这对应于 **2015 或更高版本的 Visual Studio**，或 **GCC 7.1** 版本以上。同时，SDK 还是需要一些运行库，因此它需要 **Windows 7** 以上。编译部分测试和示例可能需要 **Visual Studio 2017** 或 **GCC 8**。 

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
    unlock();
    turn_around(0.5, 1.57);
}
```

## 细节

要了解关于使用的更多细节，请浏览[开发指南](https://autolaborcenter.github.io/pm1-docs-sphinx/development/native/readme.html)。
