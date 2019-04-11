//
// Created by User on 2019/4/10.
//

#ifndef PM1_SDK_NATIVE_H
#define PM1_SDK_NATIVE_H


#ifdef  _MSC_VER
#define DLL_EXPORT __declspec(dllexport)
#define STD_CALL   __stdcall
#else
#define DLL_EXPORT
#define STD_CALL
#endif // _MSC_VER

namespace autolabor {
	namespace pm1 {
		namespace native {
			using handler_t = unsigned long;
			
			/**
			 * 获取错误信息
			 */
			DLL_EXPORT const char *STD_CALL
			get_error_info(handler_t) noexcept;
			
			/**
			 * 移除错误信息
			 */
			DLL_EXPORT void STD_CALL
			remove_error_info(handler_t) noexcept;
			
			/**
			 * 清除错误信息
			 */
			DLL_EXPORT void STD_CALL
			clear_error_info() noexcept;
			
			/**
			 * 获取当前打开的串口名字
			 */
			DLL_EXPORT const char *STD_CALL
			get_current_port() noexcept;
			
			/**
			 * 初始化
			 */
			DLL_EXPORT handler_t STD_CALL
			initialize(const char *port, double &progress) noexcept;
			
			/**
			 * 关闭
			 */
			DLL_EXPORT handler_t STD_CALL
			shutdown() noexcept;
			
			/**
			 * 获取里程计值
			 */
			DLL_EXPORT handler_t STD_CALL
			get_odometry(double &s, double &sa,
			             double &x, double &y, double &theta,
			             double &vx, double &vy, double &w) noexcept;
			
			/**
			 * 清除里程计累计值
			 */
			DLL_EXPORT handler_t STD_CALL
			reset_odometry() noexcept;
			
			/**
			 * 锁定底盘
			 */
			DLL_EXPORT handler_t STD_CALL
			lock() noexcept;
			
			/**
			 * 解锁底盘
			 */
			DLL_EXPORT handler_t STD_CALL
			unlock() noexcept;
			
			/**
			 * 检查节点状态
			 *
			 * @param what
			 *        0x00 : unknown
			 *        0x01 : normal
			 *        0xff : lock
			 *        else : error
			 */
			DLL_EXPORT handler_t STD_CALL
			check_state(unsigned char &what) noexcept;
			
			/**
			 * 行驶
			 */
			DLL_EXPORT handler_t STD_CALL
			drive(double v, double w) noexcept;
			
			/**
			 * 行驶
			 */
			DLL_EXPORT handler_t STD_CALL
			drive_spatial(double v,
			              double w,
			              double spatium,
			              double &progress) noexcept;
			
			/**
			 * 行驶
			 */
			DLL_EXPORT handler_t STD_CALL
			drive_timing(double v,
			             double w,
			             double time,
			             double &progress) noexcept;
			
			/**
			 * 暂停执行阻塞控制
			 */
			DLL_EXPORT void STD_CALL
			pause() noexcept;
			
			/**
			 * 恢复执行阻塞控制
			 */
			DLL_EXPORT void STD_CALL
			resume() noexcept;
			
			/**
			 * 取消所有正在执行的动作
			 */
			DLL_EXPORT void STD_CALL
			cancel_all() noexcept;
		}
	}
}


#endif // PM1_SDK_NATIVE_H
