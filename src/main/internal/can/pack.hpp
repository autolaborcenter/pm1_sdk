//
// Created by ydrml on 2019/3/9.
//

#ifndef PM1_SDK_PACK_HPP
#define PM1_SDK_PACK_HPP


#include <array>
#include <cstring>
#include "protocol.hpp"

namespace autolabor {
    namespace can {
        /**
         * CAN 包信息定义
         *
         * @tparam _type_t     使用的数据结构
         * @tparam _network    网络号
         * @tparam _property   优先级
         * @tparam _node_type  节点类型
         * @tparam _node_index 节点序号
         * @tparam _msg_type   消息类型
         */
        template<class _type_t,
            uint8_t _network,
            uint8_t _priority,
            uint8_t _node_type,
            uint8_t _node_index,
            uint8_t _msg_type>
        class pack_define_t {
        public:
            using type_t = _type_t;
    
            static_assert(std::is_same<_type_t, pack_with_data>::value || std::is_same<_type_t, pack_no_data>::value,
                          "struct must be a can pack type");
            static_assert(_network <= autolabor::can::mask(2), "network in 2 bits");
            static_assert(_priority <= autolabor::can::mask(3), "property in 3 bits");
            static_assert(_node_type <= autolabor::can::mask(6), "node type in 6 bits");
            static_assert(_node_index <= autolabor::can::mask(4), "node index in 4 bits");
            
            constexpr static auto network    = _network;
            constexpr static auto data_field = std::is_same<_type_t, pack_with_data>::value;
            constexpr static auto priority   = _priority;
            constexpr static auto node_type  = _node_type;
            constexpr static auto node_index = _node_index;
            constexpr static auto msg_type   = _msg_type;
    
            constexpr static type_t
                stub{0xfe,
                
                     node_type >> 4u,
                     priority,
                     data_field,
                     network,
                
                     node_index,
                     node_type & 0xfu,
                
                     msg_type};
            
            inline static bool match(const type_t &msg) {
                return msg.node_type() == node_type
                       && msg.node_index == node_index
                       && msg.msg_type == msg_type;
            }
        };
        
        /** 打包（无数据域） */
        template<class info_t>
        inline typename info_t::type_t pack(uint8_t reserve = 0) {
            static_assert(std::is_same<typename info_t::type_t, pack_no_data>::value,
                          "cannot build a signal pack with message info");
            
            typename info_t::type_t msg{};
            msg = info_t::stub;
            msg.reserve = reserve;
            fill_crc(bytes_begin(msg) + 1, bytes_end(msg));
            return msg;
        }
        
        /** 打包（有数据域） */
        template<class info_t>
        inline typename info_t::type_t pack(const std::array<uint8_t, 8> &data, uint8_t frame_id = 0) {
            static_assert(std::is_same<typename info_t::type_t, pack_with_data>::value,
                          "cannot build a message pack with signal info");
            
            typename info_t::type_t msg{};
            msg = info_t::stub;
            msg.frame_id = frame_id;
            std::memcpy(msg.data, data.data(), data.size());
            fill_crc(bytes_begin(msg) + 1, bytes_end(msg));
            return msg;
        }
    } // namespace can
} // namespace autolabor


#endif //PM1_SDK_PACK_HPP
