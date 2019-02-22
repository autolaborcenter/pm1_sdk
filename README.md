# PM1 SDK API 文档

* 命名空间 `autolabor::pm1`

## 连接控制

* `bool Initialize(std::string port)`
  * 描述：初始化
  * 参数 `port`：串口名
  * 返回：是否成功
* `bool Initialize()`
  * 描述：自动选取串口并初始化
  * 返回：是否成功
* `void Shutdown()`
  * 描述：关闭

## 行为交互

### 阻塞控制

* 走简单几何路径
  * `void GoStraight(double speed, double distance)`

    * 描述：直走
    * 参数 `speed`：线速度（m/s）
    * 参数 `distance`：行驶距离（m，非负）
  * `void GoStraight(double speed, double time)`

    - 描述：直走
    - 参数 `speed`：线速度（m/s）
    - 参数 `time`：行驶时间（s，非负）
  * `void GoArc(double speed, double r, double rad)`

    * 描述：走圆弧
    * 参数 `speed`：线速度（m/s）
    * 参数 `r`：转弯半径（m）
    * 参数 `rad`：行驶弧度（rad，非负）
  * `void GoArc(double speed, double r, double time)`

    - 描述：走圆弧
    - 参数 `speed`：线速度（m/s）
    - 参数 `r`：转弯半径（m）
    - 参数 `time`：行驶时间（s，非负）
  * `void TurnAround(double speed, double rad)`
    * 描述：原地转
    * 参数 `speed`：角速度（rad/s）
    * 参数 `rad`：弧度（rad，非负）
  * `void TurnAround(double speed, double time)`
    * 描述：原地转
    * 参数 `speed`：角速度（rad/s）
    * 参数 `time`：时间（s，非负）
* 控制
  * `void pause()`
    * 描述：暂停执行阻塞控制
  * `void resume()`

    * 描述：恢复执行阻塞控制
  * `void delay(double time)`
    * 描述：延时
    * 参数 `time`：时间（s，非负）

### 状态交互

```c++
struct Odometry{
	double x, y, yaw, vx, vy, w;
}
```

* `Odometry GetOdometry()`
  * 描述：获取里程计值
* `void Drive(double v, double w)`
  * 描述：控制机器人运行
  * 参数 `v`：线速度（m/s）
  * 参数 `w`：角速度（rad/s）
