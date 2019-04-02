//
// Created by ydrml on 2019/2/22.
//

#ifndef PM1_SDK_API_H
#define PM1_SDK_API_H


#ifdef  _MSC_VER
#define DllExport __declspec(dllexport)
#else
#define DllExport
#endif // _MSC_VER


#include <string>
#include <vector>

namespace autolabor {
	namespace pm1 {
		enum class node_state : uint8_t {
			unknown  = 0x00,
			enabled  = 0x01,
			disabled = 0xff
		};
		
		struct chassis_state {
			node_state _ecu0, _ecu1, _tcu;
		};
		
		/** 表示全局指令执行的结果 */
		template<class t>
		struct result {
			/** 错误信息 */
			const std::string error_info;
			
			t data;
			
			/** 判断结果是否是成功的 */
			explicit operator bool() const {
				return error_info.empty();
			}
		};
		
		template<>
		struct result<void> {
			/** 错误信息 */
			const std::string error_info;
			
			/** 判断结果是否是成功的 */
			explicit operator bool() const {
				return error_info.empty();
			}
		};
		
		/**
		 * @return 全部串口的名字列表
		 */
		DllExport std::vector<std::string> serial_ports();
		
		/**
		 * 初始化
		 *
		 * @param port 串口名字
		 * @returns 是否成功
		 */
		DllExport result<std::string> initialize(const std::string &port = "");
		
		/**
		 * 关闭
		 */
		DllExport result<void> shutdown();
		
		//		/**
		//		 * 走直线
		//		 *
		//		 * @param speed    线速度
		//		 * @param distance 行驶距离
		//		 */
		//		DllExport result<void> go_straight(double speed, double distance);
		//
		//		/**
		//		 * 走直线
		//		 *
		//		 * @param speed 线速度
		//		 * @param time  行驶时间
		//		 */
		//		DllExport result<void> go_straight_timing(double speed, double time);
		//
		//		/**
		//		 * 走圆弧
		//		 *
		//		 * @param speed 线速度
		//		 * @param r     转弯半径
		//		 * @param rad   行驶时间
		//		 */
		//		DllExport result<void> go_arc(double speed, double r, double rad);
		//
		//		/**
		//		 * 走圆弧
		//		 *
		//		 * @param speed 线速度
		//		 * @param r     转弯半径
		//		 * @param time  行驶时间
		//		 */
		//		DllExport result<void> go_arc_timing(double speed, double r, double time);
		//
		//		/**
		//		 * 原地转
		//		 *
		//		 * @param speed 角速度
		//		 * @param rad   弧度
		//		 */
		//		DllExport result<void> turn_around(double speed, double rad);
		//
		//		/**
		//		 * 原地转
		//		 *
		//		 * @param speed 角速度
		//		 * @param time  时间
		//		 */
		//		DllExport result<void> turn_around_timing(double speed, double time);
		//
		//		/**
		//		 * 暂停执行阻塞控制
		//		 */
		//		DllExport result<void> pause();
		//
		//		/**
		//		 * 恢复执行阻塞控制
		//		 */
		//		DllExport result<void> resume();
		//
		//		/**
		//		 * 控制机器人运行
		//		 *
		//		 * @param v 线速度
		//		 * @param w 角速度
		//		 */
		//		DllExport result<void> drive(double v, double w);
		//
		//
		//		struct odometry { double x, y, yaw, vx, vy, w; };
		//
		//		/**
		//		 * 获取里程计值
		//		 *
		//		 * @return 里程计值
		//		 */
		//		DllExport result<odometry> get_odometry();
		//
		//		/**
		//		 * 清除里程计累计值
		//		 */
		//		DllExport result<void> reset_odometry();
		//
		//		/**
		//		 * 锁定底盘
		//		 */
		//		DllExport result<void> lock();
		//
		//		/**
		//		 * 解锁底盘
		//		 */
		//		DllExport result<void> unlock();
		//
		//		/**
		//	     * 检查节点状态
		//	     */
		//		DllExport result<chassis_state> get_chassis_state();
		//
		//		/**
		//         * 延时
		//         *
		//         * @param time 时间
		//         */
		//		DllExport void delay(double time);
	}
}


#endif //PM1_SDK_API_H
