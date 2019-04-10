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

namespace autolabor::pm1::native {
	using handler_t = unsigned long;
	
	/**
	 * 获取错误信息
	 */
	DLL_EXPORT const char *STD_CALL
	get_error_info(handler_t);
	
	/**
	 * 移除错误信息
	 */
	DLL_EXPORT void STD_CALL
	remove_error_info(handler_t);
	
	/**
	 * 清除错误信息
	 */
	DLL_EXPORT void STD_CALL
	clear_error_info();
	
	DLL_EXPORT const char *STD_CALL
	get_current_port();
	
	/**
	 * 初始化
	 */
	DLL_EXPORT handler_t STD_CALL
	initialize(const char *port, double *const progress);
	
	/**
	 * 关闭
	 */
	DLL_EXPORT handler_t STD_CALL
	shutdown();
	
	/**
	 * 获取里程计值
	 */
	DLL_EXPORT handler_t STD_CALL
	get_odometry(double &s, double &sa,
	             double &x, double &y, double &theta,
	             double &vx, double &vy, double &w);
	
	/**
	 * 清除里程计累计值
	 */
	DLL_EXPORT handler_t STD_CALL
	reset_odometry();
	
	/**
	 * 锁定底盘
	 */
	DLL_EXPORT handler_t STD_CALL
	lock();
	
	/**
	 * 解锁底盘
	 */
	DLL_EXPORT handler_t STD_CALL
	unlock();
	
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
	check_state(unsigned char &what);
	
	/**
     * 行驶
     */
	DLL_EXPORT handler_t STD_CALL
	drive(double v, double w);
	
	/**
	 * 行驶
	 */
	DLL_EXPORT handler_t STD_CALL
	drive_spatial(double v, double w, double spatium, double &progress);
	
	/**
	 * 行驶
	 */
	DLL_EXPORT handler_t STD_CALL
	drive_timing(double v, double w, double time, double &progress);
	
	/**
	 * 暂停执行阻塞控制
	 */
	DLL_EXPORT void STD_CALL
	pause();
	
	/**
	 * 恢复执行阻塞控制
	 */
	DLL_EXPORT void STD_CALL
	resume();
	
	/**
	 * 取消所有正在执行的动作
	 */
	DLL_EXPORT void STD_CALL
	cancel_all();
}


#endif // PM1_SDK_NATIVE_H
