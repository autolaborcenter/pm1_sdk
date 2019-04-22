//
// Created by ydrml on 2019/2/22.
//

#ifndef PM1_SDK_API_H
#define PM1_SDK_API_H


#ifdef  _MSC_VER
#define DLL_EXPORT __declspec(dllexport)
#else
#define DLL_EXPORT
#endif // _MSC_VER


#include <string>
#include <vector>

namespace autolabor {
	namespace pm1 {
		/**
		 * 顶层函数执行的结果
		 */
		template<class t>
		struct result {
			/**
			 * 错误信息
			 *
			 * 若成功，空字符串
			 * 若失败，错误信息
			 */
			std::string error_info;
			
			/**
			 * 返回值
			 *
			 * 若成功，返回值
			 * 若失败，未定义
			 */
			t value;
			
			/**
			 * 判断顶层函数是否成功
			 */
			explicit operator bool() const {
				return error_info.empty();
			}
		};
		
		/**
		 * 特化：无返回值的顶层函数执行的结果
		 */
		template<>
		struct result<void> {
			/**
			 * 错误信息
			 *
			 * 若成功，空字符串
			 * 若失败，错误信息
			 */
			std::string error_info;
			
			/**
			 * 判断顶层函数是否成功
			 */
			explicit operator bool() const {
				return error_info.empty();
			}
		};
		
		/**
		 * 底盘初始设定项
		 */
		struct chassis_config {
			double width          = NAN,
			       length         = NAN,
			       wheel_radius   = NAN,
			       optimize_width = NAN,
			       acceleration   = NAN,
			       max_v          = NAN,
			       max_w          = NAN;
		};
		
		/**
		 * 底盘状态
		 */
		enum class chassis_state : uint8_t {
			unknown  = 0x00, // 未知（离线）
			unlocked = 0x01, // 未锁定
			error    = 0x7f, // 已连接但异常
			locked   = 0xff  // 已锁定
		};
		
		/**
		 * 里程计
		 */
		struct odometry { double x, y, yaw, vx, vy, w; };
		
		/**
		 * 获取串口列表
		 * 
		 * @return 全部串口的名字列表
		 */
		DLL_EXPORT std::vector<std::string>
		serial_ports();
		
		/**
		 * 初始化
		 *
		 * @param port     串口名字
		 * @param config   初始设定参数
		 * @param progress 进度
		 * @returns 执行结果
		 */
		DLL_EXPORT result<std::string>
		initialize(const std::string &port = "",
		           double *progress = nullptr);
		
		/**
		 * 关闭
		 *
		 * @returns 执行结果
		 */
		DLL_EXPORT result<void>
		shutdown();
		
		/**
		 * 获取里程计值
		 *
		 * @return 里程计值或异常信息
		 */
		DLL_EXPORT result<odometry>
		get_odometry();
		
		/**
		 * 清除里程计累计值
		 *
		 * @returns 执行结果
		 */
		DLL_EXPORT result<void>
		reset_odometry();
		
		/**
		 * 锁定底盘
		 *
		 * @returns 执行结果
		 */
		DLL_EXPORT result<void>
		lock();
		
		/**
		 * 解锁底盘
		 *
		 * @returns 执行结果
		 */
		DLL_EXPORT result<void>
		unlock();
		
		/**
		 * 检查节点状态
		 *
		 * @returns 执行结果
		 */
		DLL_EXPORT chassis_state
		check_state();
		
		/**
		 * 延时
		 *
		 * @param time 时间
		 */
		DLL_EXPORT void
		delay(double time);
		
		/**
 		 * 控制机器人运行
 		 *
 		 * @param speed  轮速
 		 * @param rudder 舵轮转角
 		 */
		DLL_EXPORT result<void>
		drive_physical(double speed, double rudder);
		
		/**
		 * 控制机器人运行
		 *
		 * @param left  左轮角速度
		 * @param right 右轮角速度
		 */
		DLL_EXPORT result<void>
		drive_wheels(double left, double right);
		
		/**
		 * 控制机器人运行
		 *
		 * @param v 线速度
		 * @param w 角速度
		 */
		DLL_EXPORT result<void>
		drive_velocity(double v, double w);
		
		/**
		 * 控制机器人运行
		 *
		 * @param v 线速度
		 * @param w 角速度
		 */
		DLL_EXPORT result<void>
		drive(double v, double w);
		
		/**
		 * 控制机器人按空间约束运行指定动作
		 */
		DLL_EXPORT result<void>
		drive_spatial(double v,
		              double w,
		              double spatium,
		              double *progress);
		
		/**
		 * 控制机器人按时间约束运行指定动作
		 */
		DLL_EXPORT result<void>
		drive_timing(double v,
		             double w,
		             double time,
		             double *progress);
		
		/**
		 * 走直线
		 *
		 * @param speed  线速度
		 * @param meters 距离
		 */
		DLL_EXPORT result<void>
		go_straight(double speed,
		            double meters,
		            double *progress = nullptr);
		
		/**
		 * 走直线
		 *
		 * @param speed   线速度
		 * @param seconds 时间
		 */
		DLL_EXPORT result<void>
		go_straight_timing(double speed,
		                   double seconds,
		                   double *progress = nullptr);
		
		/**
		 * 原地转
		 *
		 * @param speed 角速度
		 * @param rad   弧度
		 */
		DLL_EXPORT result<void>
		turn_around(double speed,
		            double rad,
		            double *progress = nullptr);
		
		/**
		 * 原地转
		 *
		 * @param speed 角速度
		 * @param time  时间
		 */
		DLL_EXPORT result<void>
		turn_around_timing(double speed,
		                   double time,
		                   double *progress = nullptr);
		
		/**
		 * 走圆弧
		 *
		 * @param progress 进度
		 */
		DLL_EXPORT result<void>
		go_arc_vs(double v,
		          double r,
		          double s,
		          double *progress = nullptr);
		/**
		 * 走圆弧
		 *
		 * @param progress 进度
		 */
		DLL_EXPORT result<void>
		go_arc_va(double v,
		          double r,
		          double a,
		          double *progress = nullptr);
		/**
		 * 走圆弧
		 *
		 * @param progress 进度
		 */
		DLL_EXPORT result<void>
		go_arc_ws(double w,
		          double r,
		          double s,
		          double *progress = nullptr);
		/**
		 * 走圆弧
		 *
		 * @param progress 进度
		 */
		DLL_EXPORT result<void>
		go_arc_wa(double w,
		          double r,
		          double a,
		          double *progress = nullptr);
		/**
		 * 走圆弧
		 *
		 * @param progress 进度
		 */
		DLL_EXPORT result<void>
		go_arc_vt(double v,
		          double r,
		          double t,
		          double *progress = nullptr);
		/**
		 * 走圆弧
		 *
		 * @param progress 进度
		 */
		DLL_EXPORT result<void>
		go_arc_wt(double w,
		          double r,
		          double t,
		          double *progress = nullptr);
		
		/**
		 * 暂停执行阻塞控制
		 */
		DLL_EXPORT void
		pause();
		
		/**
		 * 恢复执行阻塞控制
		 */
		DLL_EXPORT void
		resume();
		
		/**
		 * 检查是否暂停
		 *
		 * @return 是否暂停
		 */
		DLL_EXPORT bool
		is_paused();
		
		/**
		 * 取消所有正在执行的动作
		 */
		DLL_EXPORT void
		cancel_action();
	}
}


#endif //PM1_SDK_API_H
