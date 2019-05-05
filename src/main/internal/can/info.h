﻿//
// Created by ydrml on 2019/3/9.
//

#ifndef PM1_SDK_INFO_H
#define PM1_SDK_INFO_H

#include <array>
#include "pack.h"

namespace autolabor {
    namespace can {
        using sgn = autolabor::can::union_no_data;   // 信号，无数据
        using msg = autolabor::can::union_with_data; // 消息，有数据
        
        /**
         * CAN 包信息定义
         *
         * @tparam _data_t     使用的数据结构
         * @tparam _network    网络号
         * @tparam _property   优先级
         * @tparam _node_type  节点类型
         * @tparam _node_index 节点序号
         * @tparam _msg_type   消息类型
         */
        template<class _data_t,
            uint8_t _network,
            uint8_t _priority,
            uint8_t _node_type,
            uint8_t _node_index,
            uint8_t _msg_type>
        class info {
        public:
            using data_t = _data_t;
    
            static_assert(std::is_same<_data_t, sgn>::value || std::is_same<_data_t, msg>::value,
                          "struct must be a can pack type");
            static_assert(_network <= autolabor::can::mask(2), "network in 2 bits");
            static_assert(_priority <= autolabor::can::mask(3), "property in 3 bits");
            static_assert(_node_type <= autolabor::can::mask(6), "node type in 6 bits");
            static_assert(_node_index <= autolabor::can::mask(4), "node index in 4 bits");
            
            constexpr static auto network    = _network;
            constexpr static auto data_field = std::is_same<_data_t, msg>::value;
            constexpr static auto priority   = _priority;
            constexpr static auto node_type  = _node_type;
            constexpr static auto node_index = _node_index;
            constexpr static auto msg_type   = _msg_type;
    
            constexpr static decltype(data_t::data)
                stub{0xfe,
        
                     node_type >> 4u,
                     priority,
                     data_field,
                     network,
        
                     node_index,
                     node_type & 0xfu,
                     msg_type};
            
            inline static bool match(const data_t &msg) {
                return msg.data.node_type() == node_type
                       && msg.data.node_index == node_index
                       && msg.data.msg_type == msg_type;
            }
        };
        
        /** 打包（无数据域） */
        template<class info_t>
        inline typename info_t::data_t pack(uint8_t reserve = 0) {
            static_assert(std::is_same<info_t::data_t, sgn>::value,
                          "cannot build a signal pack with message info");
    
            info_t::data_t msg{};
            msg.data         = info_t::stub;
            msg.data.reserve = reserve;
            reformat(msg);
            return msg;
        }
        
        /** 打包（有数据域） */
        template<class info_t>
        inline typename info_t::data_t pack(const std::array<uint8_t, 8> &data, uint8_t frame_id = 0) {
            static_assert(std::is_same<info_t::data_t, msg>::value,
                          "cannot build a message pack with signal info");
    
            info_t::data_t msg{};
            msg.data          = info_t::stub;
            msg.data.frame_id = frame_id;
            std::memcpy(msg.data.data, data.data(), data.size());
            reformat(msg);
            return msg;
        }
    } // namespace can
} // namespace autolabor

#endif //PM1_SDK_INFO_H
