//
// Created by User on 2019/4/10.
//

#ifndef PM1_SDK_NATIVE_H
#define PM1_SDK_NATIVE_H

#ifdef _MSC_VER
#define DLL_EXPORT extern "C" __declspec(dllexport)
#define STD_CALL __stdcall
#else
#define DLL_EXPORT
#define STD_CALL __artribute__((stdcall))
#endif // _MSC_VER

namespace autolabor {
    namespace pm1 {
        namespace native {
            using handler_t = unsigned int;
            
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
            initialize(const char *port,
                       double &progress) noexcept;
            
            /**
             * 关闭
             */
            DLL_EXPORT handler_t STD_CALL
            shutdown() noexcept;
            
            /**
             * 获取默认参数
             */
            DLL_EXPORT double STD_CALL
            get_default_parameter(handler_t id) noexcept;
            
            /**
             * 获取参数
             */
            DLL_EXPORT handler_t STD_CALL
            get_parameter(handler_t id, double &value) noexcept;
            
            /**
             * 设置参数
             */
            DLL_EXPORT handler_t STD_CALL
            set_parameter(handler_t id, double value) noexcept;
            
            /**
             * 重置参数
             */
            DLL_EXPORT handler_t STD_CALL
            reset_parameter(handler_t id) noexcept;
            
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
             * @return 0x00 : offline
             *         0x01 : unlocked
             *         0xff : locked
             *         else : error
             */
            DLL_EXPORT unsigned char STD_CALL
            check_state() noexcept;
            
            /**
             * 按物理模型参数设置目标控制量
             */
            DLL_EXPORT handler_t STD_CALL
            drive_physical(double speed, double rudder) noexcept;
            
            /**
             * 按两轮轮速设置目标控制量
             */
            DLL_EXPORT handler_t STD_CALL
            drive_wheels(double left, double right) noexcept;
            
            /**
             * 按速度矢量设置目标控制量
             */
            DLL_EXPORT handler_t STD_CALL
            drive_velocity(double v, double w) noexcept;
            
            /**
             * 计算里程度量
             */
            DLL_EXPORT double STD_CALL
            calculate_spatium(double spatium, double angle) noexcept;
            
            /**
             * 按里程度量约束行驶
             */
            DLL_EXPORT handler_t STD_CALL
            drive_spatial(double v,
                          double w,
                          double spatium,
                          double &progress) noexcept;
            
            /**
             * 按时间约束行驶
             */
            DLL_EXPORT handler_t STD_CALL
            drive_timing(double v,
                         double w,
                         double time,
                         double &progress) noexcept;
            
            /**
             * 矫正后轮
             */
            DLL_EXPORT handler_t STD_CALL
            adjust_rudder(double offset,
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
             * 检查当前是否已暂停
             */
            DLL_EXPORT bool STD_CALL
            is_paused() noexcept;
            
            /**
             * 取消正在执行的动作
             *
             * 当成功获取锁，认为所有动作已取消
             */
            DLL_EXPORT void STD_CALL
            cancel_action() noexcept;
        } // namespace native
    } // namespace pm1
} // namespace autolabor

#endif // PM1_SDK_NATIVE_H
