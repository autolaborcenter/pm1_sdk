//
// Created by ydrml on 2019/2/25.
//

#ifndef PM1_SDK_INFO_H
#define PM1_SDK_INFO_H


#include <sstream>
#include <numeric>

#include "../../utilities/serial_parser/memory.hpp"

namespace autolabor {
    namespace can {
        /**
         * 转换器
         * @tparam t 内容类型
         */
        template<class t>
        union msg_union {
            uint8_t bytes[sizeof(t)];
            t       data;
        };
    
        /** 计算某个长度的位遮盖 */
        constexpr inline uint8_t mask(uint8_t length) { return 0xffu >> (8u - length); }
        
        /** 无数据域 CAN 包 */
        struct pack_no_data {
            uint8_t
                head,
    
                node_type_h : 2, //
                priority    : 3; // 第 1 个信息字节
            bool
                payload     : 1; // 从低到高
            uint8_t
                network     : 2, //
                
                node_index  : 4, // 第 2 个信息字节
                node_type_l : 4, // 从低到高
                
                msg_type,
                reserve,
                crc;
    
            [[nodiscard]] inline uint8_t node_type() const {
                return static_cast<uint8_t>(node_type_h << 4u) | node_type_l;
            }
    
            [[nodiscard]] inline std::string to_string() const;
        };
        
        /** 有数据域 CAN 包 */
        struct pack_with_data {
            uint8_t
                head,
    
                node_type_h : 2, //
                priority    : 3; // 第 1 个信息字节
            bool
                payload     : 1; // 从低到高
            uint8_t
                network     : 2, //
    
                node_index  : 4, // 第 2 个信息字节
                node_type_l : 4, // 从低到高
                
                msg_type,
                frame_id,
                data[8],
                crc;
    
            [[nodiscard]] inline uint8_t node_type() const {
                return static_cast<uint8_t>(node_type_h << 4u) | node_type_l;
            }
    
            [[nodiscard]] inline std::string to_string() const;
        };
        
        /** 无数据域 CAN 包转换器 */
        using union_no_data = msg_union<pack_no_data>;
        
        /** 有数据域 CAN 包转换器 */
        using union_with_data = msg_union<pack_with_data>;
    
        /**
         * 循环冗余计算
         *
         * @param begin 参与循环冗余计算的起点迭代器
         * @param end   参与循环冗余计算的终点迭代器
         * @return 校验码
         */
        template<class t>
        uint8_t crc_calculate(t begin, t end) {
            constexpr static uint8_t crc8[]{
                0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
                157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
                35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
                190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
                70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
                219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
                101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
                248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
                140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
                17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
                175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
                50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
                202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
                87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
                233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
                116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53};
        
            return std::accumulate(begin, end, static_cast<uint8_t>(0),
                                   [&](uint8_t sum, uint8_t it) { return crc8[sum ^ it]; });
        
        }
    
        /**
         * 循环冗余校验
         *
         * @param begin 参与循环冗余计算的起点迭代器
         * @param end   参与循环冗余计算的终点迭代器（包括校验字节）
         * @return 是否通过检验
         */
        template<class t>
        inline bool crc_check(t begin, t end) {
            auto last = end - 1;
            return *last == crc_calculate(begin, last);
        }
    
        /**
         * 填充校验和
         *
         * @param begin 参与循环冗余计算的起点迭代器
         * @param end   参与循环冗余计算的终点迭代器（包括校验字节）
         */
        template<class t>
        inline void fill_crc(t begin, t end) {
            auto last = end - 1;
            *last = crc_calculate(begin, last);
        }
    
        std::string pack_no_data::to_string() const {
            std::stringstream builder;
            builder << std::hex
                    << "head:       0x" << +head << std::endl
                    << "network:    0x" << +network << std::endl
                    << std::boolalpha
                    << "data_field: " << payload << std::endl
                    << std::dec
                    << "property:   " << +priority << std::endl
                    << std::hex
                    << "node_type:  0x" << +node_type() << std::endl
                    << std::dec
                    << "node_index: " << +node_index << std::endl
                    << std::hex
                    << "msg_type:   0x" << +msg_type << std::endl
                    << "reserve:    0x" << +reserve << std::endl
                    << "crc:        0x" << +crc << std::endl
                    << "crc_check:  " << crc_check(bytes_begin(*this) + 1, bytes_end(*this)) << std::endl;
            return builder.str();
        }
    
        std::string pack_with_data::to_string() const {
            std::stringstream builder;
            builder << std::hex
                    << "head:       0x" << +head << std::endl
                    << "network:    0x" << +network << std::endl
                    << std::boolalpha
                    << "data_field: " << payload << std::endl
                    << std::dec
                    << "property:   " << +priority << std::endl
                    << std::hex
                    << "node_type:  0x" << +node_type() << std::endl
                    << std::dec
                    << "node_index: " << +node_index << std::endl
                    << std::hex
                    << "msg_type:   0x" << +msg_type << std::endl
                    << "payload:    ";
            for (int b : data) {
                if (b < 0x10)
                    builder << '0';
                builder << b << " ";
            }
            builder << "]" << std::endl
                    << "crc:        0x" << +crc << std::endl
                    << "crc_check:  " << crc_check(bytes_begin(*this) + 1, bytes_end(*this)) << std::endl;
            return builder.str();
        }
    } // namespace can
} // namespace autolabor

#endif //PM1_SDK_INFO_H
