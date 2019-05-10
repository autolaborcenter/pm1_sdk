//
// Created by ydrml on 2019/2/26.
//

#ifndef PM1_SDK_CAN_DEFINE_H
#define PM1_SDK_CAN_DEFINE_H

#include "can/pack.h"

using namespace autolabor::can;

namespace autolabor {
    namespace pm1 {
        /**
         * 广播地址常量
         */
        constexpr uint8_t any_type  = 0x3f,
                          any_index = 0x0f;
        
        /** 任意类型控制器结构 */
        struct any_controller {
            constexpr static uint8_t type_id    = any_type;
            constexpr static uint8_t node_index = any_index;
        };
    
        /** 问答结构 */
        template<class node, uint8_t _msg_type>
        struct dialog {
            using sgn = pack_define_t<sgn, 0, 0, node::type_id, node::node_index, _msg_type>;
            using msg = pack_define_t<msg, 0, 0, node::type_id, node::node_index, _msg_type>;
    
            using tx  = sgn;
            using rx  = msg;
        };
        
        /** 通用控制器消息 */
        template<class type = any_controller>
        struct unit {
            constexpr static uint8_t type_id    = type::type_id;
            constexpr static uint8_t node_index = type::node_index;
            
            // 状态
            using state_tx       = typename dialog<unit, 0x80>::tx;
            using state_rx       = typename dialog<unit, 0x80>::rx;
            // 版本 id
            using version_id_tx  = typename dialog<unit, 0x81>::tx;
            using version_id_rx  = typename dialog<unit, 0x81>::rx;
            // 设备 id
            using device_id_tx   = typename dialog<unit, 0x82>::tx;
            using device_id_rx   = typename dialog<unit, 0x82>::rx;
            // 芯片 id
            using chip_id_tx     = typename dialog<unit, 0x83>::tx;
            using chip_id_rx     = typename dialog<unit, 0x83>::rx;
            // HAL 版本
            using hal_version_tx = typename dialog<unit, 0x84>::tx;
            using hal_version_rx = typename dialog<unit, 0x84>::rx;
            // 核心板硬件版本
            using core_hardware_version_tx  = typename dialog<unit, 0x85>::tx;
            using core_hardware_version_rx  = typename dialog<unit, 0x85>::rx;
            // 扩展板硬件版本
            using extra_hardware_version_tx = typename dialog<unit, 0x86>::tx;
            using extra_hardware_version_rx = typename dialog<unit, 0x86>::rx;
            // 软件版本
            using software_version_tx = typename dialog<unit, 0x87>::tx;
            using software_version_rx = typename dialog<unit, 0x87>::rx;
            // 累计运行时间
            using uptime_tx      = typename dialog<unit, 0x88>::tx;
            using uptime_rx      = typename dialog<unit, 0x88>::rx;
            // 紧急停止
            using emergency_stop = typename dialog<unit, 0xff>::tx;
            using release_stop   = typename dialog<unit, 0xff>::rx;
        };
    
        /** 整车控制器包信息协议 */
        template<uint8_t _node_index = any_index>
        class vcu {
        public:
            constexpr static uint8_t type_id    = 0x10;
            constexpr static uint8_t node_index = _node_index;
    
            // 电池
            using battery_persent_tx  = typename dialog<vcu, 0x1>::tx;
            using battery_persent_rx  = typename dialog<vcu, 0x1>::rx;
            using battery_time_tx     = typename dialog<vcu, 0x2>::tx;
            using battery_time_rx     = typename dialog<vcu, 0x2>::rx;
            using battery_quantity_tx = typename dialog<vcu, 0x3>::tx;
            using battery_quantity_rx = typename dialog<vcu, 0x3>::rx;
            using battery_voltage_tx  = typename dialog<vcu, 0x4>::tx;
            using battery_voltage_rx  = typename dialog<vcu, 0x4>::rx;
            using battery_current_tx  = typename dialog<vcu, 0x5>::tx;
            using battery_current_rx  = typename dialog<vcu, 0x5>::rx;
            // 急停开关
            using power_switch_tx     = typename dialog<vcu, 0x5>::tx;
            using power_switch_rx     = typename dialog<vcu, 0x5>::rx;
        };
        
        /** 动力控制器包信息协议 */
        template<uint8_t _node_index = any_index>
        class ecu {
        public:
            constexpr static uint8_t type_id    = 0x11;
            constexpr static uint8_t node_index = _node_index;
            // 目标速度
            using target_speed        = typename dialog<ecu, 0x1>::msg;
            // 当前速度
            using current_speed_tx    = typename dialog<ecu, 0x5>::tx;
            using current_speed_rx    = typename dialog<ecu, 0x5>::rx;
            // 当前编码器读数
            using current_position_tx = typename dialog<ecu, 0x6>::tx;
            using current_position_rx = typename dialog<ecu, 0x6>::rx;
            // 编码器清零
            using clear               = typename dialog<ecu, 0x7>::sgn;
            // 超时时间
            using timeout             = typename dialog<ecu, 0xa>::msg;
        };
    
        /** 转向控制器包信息协议 */
        template<uint8_t _node_index = any_index>
        class tcu {
        public:
            constexpr static uint8_t type_id    = 0x12;
            constexpr static uint8_t node_index = _node_index;
            // 目标角度
            using target_position     = typename dialog<tcu, 0x1>::msg;
            // 目标角度增量
            using target_difference   = typename dialog<tcu, 0x2>::msg;
            // 当前角度
            using current_position_tx = typename dialog<tcu, 0x3>::tx;
            using current_position_rx = typename dialog<tcu, 0x3>::rx;
            // 目标角度增量
            using target_speed        = typename dialog<tcu, 0x4>::msg;
            // 当前速度
            using current_speed_tx    = typename dialog<tcu, 0x5>::tx;
            using current_speed_rx    = typename dialog<tcu, 0x5>::rx;
            // 编码器复位
            using encoder_reset       = typename dialog<tcu, 0x6>::sgn;
            // 超时时间
            using timeout             = typename dialog<tcu, 0x7>::msg;
        };
    
        /**
         * 从消息数据包提取逐字节序列化的数据
         *
         * @tparam data_t 数据类型
         *
         * @param msg 消息数据包
         * @return 数据值
         */
        template<class data_t>
        inline auto get_big_endian(const union_with_data &msg)
        -> typename std::decay<data_t>::type {
            using actual_type = typename std::decay<data_t>::type;
            static_assert(sizeof(actual_type) <= 8, "a pack cannot load more than 8 bytes");
        
            msg_union<actual_type> buffer{};
            std::reverse_copy(msg.data.data, msg.data.data + sizeof(actual_type), buffer.bytes);
            return buffer.data;
        }
    
        /**
         * 将数据逐字节序列化并写入消息数据包
         *
         * @tparam pack_info_t 包类型
         * @tparam data_t      数据类型
         *
         * @param value 数据值
         * @return 消息数据包
         */
        template<class pack_info_t, class data_t>
        inline auto pack_big_endian(data_t value)
        -> decltype(pack<pack_info_t>()) {
            using actual_type = typename std::decay<data_t>::type;
            static_assert(sizeof(actual_type) <= 8, "a pack cannot load more than 8 bytes");
        
            msg_union<actual_type> buffer1{};
            std::array<uint8_t, 8> buffer2{};
        
            buffer1.data = value;
            std::reverse_copy(buffer1.bytes, buffer1.bytes + sizeof(actual_type), buffer2.data());
            
            return pack<pack_info_t>(buffer2);
        }
        
        /** 节点状态 */
        enum class node_state_t : uint8_t {
            unknown  = 0x00,
            enabled  = 0x01,
            disabled = 0xff
        };
        
        /** 判断节点状态 */
        inline node_state_t parse_state(uint8_t data) {
            switch (data) {
                case 0x01:
                    return node_state_t::enabled;
                case 0xff:
                    return node_state_t::disabled;
                default:
                    return node_state_t::unknown;
            }
        }
    }
}

#endif //PM1_SDK_CAN_DEFINE_H
