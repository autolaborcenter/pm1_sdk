﻿//
// Created by User on 2019/4/10.
//

#ifndef PM1_SDK_NATIVE_H
#define PM1_SDK_NATIVE_H

#ifdef _MSC_VER
#define DLL_EXPORT extern "C" __declspec(dllexport)
#define STD_CALL __stdcall
#else
#define DLL_EXPORT extern "C"
#define STD_CALL
#endif // _MSC_VER

namespace autolabor {
    namespace pm1 {
        namespace native {
            using handler_t = unsigned int;
            
            /**
             * 从错误管理器获取错误信息
             */
            DLL_EXPORT const char *STD_CALL
            get_error_info(handler_t) noexcept;
            
            /**
             * 从错误管理器移除错误信息
             */
            DLL_EXPORT void STD_CALL
            remove_error_info(handler_t) noexcept;
            
            /**
             * 从错误管理器清除所有错误信息
             */
            DLL_EXPORT void STD_CALL
            clear_error_info() noexcept;
            
            /**
             * 获取当前打开的串口名字
             */
            DLL_EXPORT const char *STD_CALL
            get_connected_port() noexcept;
    
            /**
             * 初始化（指针版）
             */
            DLL_EXPORT handler_t STD_CALL
            initialize_c(const char *port,
                         double *progress) noexcept;
            
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
             * 获取参数默认值
             */
            DLL_EXPORT double STD_CALL
            get_default_parameter(handler_t id) noexcept;
    
            /**
             * 获取参数当前值（指针版）
             */
            DLL_EXPORT handler_t STD_CALL
            get_parameter_c(handler_t id, double *value) noexcept;
            
            /**
             * 获取参数当前值
             */
            DLL_EXPORT handler_t STD_CALL
            get_parameter(handler_t id, double &value) noexcept;
            
            /**
             * 设置参数
             */
            DLL_EXPORT handler_t STD_CALL
            set_parameter(handler_t id, double value) noexcept;
            
            /**
             * 重置参数值到默认值
             */
            DLL_EXPORT handler_t STD_CALL
            reset_parameter(handler_t id) noexcept;
    
            /**
             * 获取电池电量百分比
             */
            DLL_EXPORT handler_t STD_CALL
            get_battery_percent(double &battery_percent) noexcept;
            
            /**
             * 获取后轮方向角（指针版）
             */
            DLL_EXPORT handler_t STD_CALL
            get_rudder_c(double *rudder) noexcept;
    
            /**
             * 获取后轮方向角
             */
            DLL_EXPORT handler_t STD_CALL
            get_rudder(double &rudder) noexcept;
            
            /**
             * 获取里程计值（指针版）
             */
            DLL_EXPORT handler_t STD_CALL
            get_odometry_c(double *stamp,
                           double *s, double *a,
                           double *x, double *y, double *theta) noexcept;
            
            /**
             * 获取里程计值
             */
            DLL_EXPORT handler_t STD_CALL
            get_odometry(double &stamp,
                         double &s, double &a,
                         double &x, double &y, double &theta) noexcept;
            
            /**
             * 清除里程计累计值
             */
            DLL_EXPORT handler_t STD_CALL
            reset_odometry() noexcept;
    
            /**
             * 开关指令发送
             */
            DLL_EXPORT handler_t STD_CALL
            set_command_enabled(bool) noexcept;
            
            /**
             * 设置使能状态
             */
            DLL_EXPORT handler_t STD_CALL
            set_enabled(bool) noexcept;
            
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
            calculate_spatium(double spatium,
                              double angle,
                              double width) noexcept;
            
            /**
             * 按里程度量约束行驶
             */
            DLL_EXPORT handler_t STD_CALL
            drive_spatial(double v,
                          double w,
                          double spatium,
                          double angle,
                          double &progress) noexcept;
    
            /**
             * 按里程度量约束行驶（指针版）
             */
            DLL_EXPORT handler_t STD_CALL
            drive_spatial_c(double v,
                            double w,
                            double spatium,
                            double angle,
                            double *progress) noexcept;
            
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
             * 控制暂停状态
             */
            DLL_EXPORT void STD_CALL
            set_paused(bool) noexcept;
            
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
