# PM1 SDK

## 介绍

本工程是 Autolabor PM1 底盘简单、跨平台、无依赖的驱动库。包括建立串口连接、控制底盘运动到关闭连接释放资源的全部流程。

## 开始使用

* 从 [`release`](https://github.com/autolaborcenter/pm1_sdk/releases) 下载库文件
* 或从[这里](https://github.com/autolaborcenter/pm1_driver_ros)下载 ROS 驱动

## API 参考

* 命名空间 `autolabor::pm1`
* 反馈数据结构

```c++
/** 表示全局指令执行的结果 */
struct result {
	/** 错误信息 */
	const std::string error_info;

    /** 判断结果是否是成功的 */
	explicit operator bool() const;
};
```

### 连接控制

* `std::vector<std::string> serial_ports()`
  * 描述：列出当前系统上所有串口的名字

  * 返回：串口名字列表

* `result initialize(std::string port = "")`

  * 描述：初始化

  * 参数 `port`：串口名，若为空，选取任意一个可用的串口

  * 返回：指令执行的结果

* `result shutdown()`

  * 描述：关闭

  * 返回：指令执行的结果

### 行为交互

#### 阻塞控制

* 走简单几何路径

  * `result go_straight(double speed, double distance)`

    * 描述：直走

    * 参数 `speed`：线速度（m/s）

    * 参数 `distance`：行驶距离（m，非负）

    * 返回：指令执行的结果

  * `result go_straight_timing(double speed, double time)`

    * 描述：直走

    * 参数 `speed`：线速度（m/s）

    * 参数 `time`：行驶时间（s，非负）

    * 返回：指令执行的结果

  * `result go_arc(double speed, double r, double rad)`

    * 描述：走圆弧

    * 参数 `speed`：线速度（m/s）

    * 参数 `r`：转弯半径（m）

    * 参数 `rad`：行驶弧度（rad，非负）

    * 返回：指令执行的结果

  * `result go_arc_timing(double speed, double r, double time)`

    - 描述：走圆弧

    - 参数 `speed`：线速度（m/s）

    - 参数 `r`：转弯半径（m）

    - 参数 `time`：行驶时间（s，非负）

    * 返回：指令执行的结果

  * `result turn_around(double speed, double rad)`

    * 描述：原地转

    * 参数 `speed`：角速度（rad/s）

    * 参数 `rad`：弧度（rad，非负）

    * 返回：指令执行的结果

  * `result turn_around_timing(double speed, double time)`

    * 描述：原地转

    * 参数 `speed`：角速度（rad/s）

    * 参数 `time`：时间（s，非负）

    * 返回：指令执行的结果

* 过程控制

  * `result pause()`

    * 描述：暂停执行阻塞控制

    * 返回：指令执行的结果

  * `result resume()`

    * 描述：恢复执行阻塞控制

    * 返回：指令执行的结果

  * `void delay(double time)`

    * 描述：延时

    * 参数 `time`：时间（s，非负）

#### 状态交互

```c++
/** 轮速里程计数据结构 */
struct Odometry { double x, y, yaw, vx, vy, w; };
```

* `Odometry GetOdometry()`

  * 描述：获取里程计值

  * 返回：里程计的计量值

* `result drive(double v, double w)`

  * 描述：控制机器人运行

  * 参数 `v`：线速度（m/s）

  * 参数 `w`：角速度（rad/s）

  * 返回：指令执行的结果
