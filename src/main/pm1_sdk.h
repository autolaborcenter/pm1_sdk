//
// Created by ydrml on 2019/2/22.
//

#ifndef PM1_SDK_API_H
#define PM1_SDK_API_H


#include <string>
#include <vector>
#include "internal/chassis.hh"

namespace autolabor {
	namespace pm1 {
		/** 表示全局指令执行的结果 */
		struct result {
			/** 错误信息 */
			const std::string error_info;
			
			/** 判断结果是否是成功的 */
			explicit operator bool() const;
		};
		
		/**
		 * @return 全部串口的名字列表
		 */
		std::vector<std::string> serial_ports();
		
		/**
		 * 初始化
		 *
		 * @param port 串口名字
		 * @returns 是否成功
		 */
		result initialize(const std::string &port = "");
		
		/**
		 * 关闭
		 */
		result shutdown();
		
		/**
		 * 走直线
		 *
		 * @param speed    线速度（m/s）
		 * @param distance 行驶距离（m，非负）
		 */
		result go_straight(double speed, double distance);
		
		/**
		 * 走直线
		 *
		 * @param speed 线速度（m/s）
		 * @param time  行驶时间（s，非负）
		 */
		result go_straight_timing(double speed, double time);
		
		/**
		 * 走圆弧
		 *
		 * @param speed 线速度（m/s）
		 * @param r     转弯半径（m，非负）
		 * @param rad   行驶时间（s，非负）
		 */
		result go_arc(double speed, double r, double rad);
		
		/**
		 * 走圆弧
		 *
		 * @param speed 线速度（m/s）
		 * @param r     转弯半径（m，非负）
		 * @param time  行驶时间（s，非负）
		 */
		result go_arc_timing(double speed, double r, double time);
		
		/**
		 * 原地转
		 *
		 * @param speed 角速度（rad/s）
		 * @param rad   弧度（rad，非负）
		 */
		result turn_around(double speed, double rad);
		
		/**
		 * 原地转
		 *
		 * @param speed 角速度（rad/s）
		 * @param time  时间（s，非负）
		 */
		result turn_around_timing(double speed, double time);
		
		/**
		 * 暂停执行阻塞控制
		 */
		result pause();
		
		/**
		 * 恢复执行阻塞控制
		 */
		result resume();
		
		/**
		 * 控制机器人运行
		 *
		 * @param v 线速度（m/s）
		 * @param w 角速度（rad/s）
		 */
		result drive(double v, double w);
		
		/**
	     * 延时
	     *
	     * @param time 时间（s，非负）
	     */
		void delay(double time);
		
		struct odometry { double x, y, yaw, vx, vy, w; };
		
		/**
		 * 获取里程计值
		 *
		 * @return 里程计值
		 */
		odometry get_odometry();
		
		/**
		 * 电机数据
		 */
		std::vector<motor_t<>> get_motors();
	}
}


#endif //PM1_SDK_API_H
