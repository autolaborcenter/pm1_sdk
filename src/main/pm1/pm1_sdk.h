//
// Created by ydrml on 2019/2/22.
//

#ifndef PM1_SDK_API_H
#define PM1_SDK_API_H


#include <string>

namespace autolabor {
	namespace pm1 {
		/**
		 * 初始化
		 *
		 * @param port 串口名字
		 * @returns 是否成功
		 */
		bool initialize(const std::string &port);
		
		/**
		 * 自动选取串口并初始化
		 *
		 * @return 是否成功
		 */
		bool initialize();
		
		/**
		 * 关闭
		 */
		void shutdown();
		
		/**
		 * 走直线
		 *
		 * @param speed    线速度（m/s）
		 * @param distance 行驶距离（m，非负）
		 */
		void go_straight(double speed, double distance);
		
		/**
		 * 走直线
		 *
		 * @param speed 线速度（m/s）
		 * @param time  行驶时间（s，非负）
		 */
		void go_straight_timing(double speed, double time);
		
		/**
		 * 走圆弧
		 *
		 * @param speed 线速度（m/s）
		 * @param r     转弯半径（m，非负）
		 * @param rad   行驶时间（s，非负）
		 */
		void go_arc(double speed, double r, double rad);
		
		/**
		 * 走圆弧
		 *
		 * @param speed 线速度（m/s）
		 * @param r     转弯半径（m，非负）
		 * @param time  行驶时间（s，非负）
		 */
		void go_arc_timing(double speed, double r, double time);
		
		/**
		 * 原地转
		 *
		 * @param speed 角速度（rad/s）
		 * @param rad   弧度（rad，非负）
		 */
		void turn_around(double speed, double rad);
		
		/**
		 * 原地转
		 *
		 * @param speed 角速度（rad/s）
		 * @param time  时间（s，非负）
		 */
		void turn_around_timing(double speed, double time);
		
		/**
		 * 暂停执行阻塞控制
		 */
		void pause();
		
		/**
		 * 恢复执行阻塞控制
		 */
		void resume();
		
		/**
		 * 延时
		 *
		 * @param time 时间（s，非负）
		 */
		void delay(double time);
		
		/**
		 * 里程计数据结构体
		 */
		struct Odometry { double x, y, yaw, vx, vy, w; };
		
		/**
		 * 获取里程计值
		 *
		 * @return 里程计值
		 */
		Odometry get_odometry();
		
		/**
		 * 控制机器人运行
		 *
		 * @param v 线速度（m/s）
		 * @param w 角速度（rad/s）
		 */
		void drive(double v, double w);
	}
}


#endif //PM1_SDK_API_H
