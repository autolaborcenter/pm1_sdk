//
// Created by ydrml on 2019/7/13.
//

#ifndef PM1_SDK_PARSER_T_HPP
#define PM1_SDK_PARSER_T_HPP


#include "protocol.hpp"

namespace marvelmind {
    /** 解析器 */
    struct parser_t {
        /** 字类型（作为解析器） */
        using word_t = uint8_t;
        
        /** 结果类型 */
        enum class result_type_t : uint8_t {
            /** 无 */
                nothing,
            /** 校验错误 */
                failed,
            /** 接收完成 */
                success
        };
        
        /** 结果结构体 */
        struct result_t {
            /** 结果类型 */
            result_type_t       type;
            std::vector<word_t> buffer;
        };
        
        /**
         * 执行解析
         *
         * @tparam iterator_t 输入迭代器类型
         * @param begin 缓存起点迭代器 -> 下一帧起点迭代器
         * @param end   缓存终点迭代器 -> 本次解析终点迭代器
         * @return 结果类型
         */
        template<class iterator_t>
        result_t operator()(iterator_t &begin, iterator_t &end) const {
            // 找到一个帧头
            while (true) {
                if (end - begin < 7)
                    return {result_type_t::nothing};
                if (destination_address_of(begin) == destination_address
                    && packet_type_of(begin) == packet_type)
                    break;
                ++begin;
            }
            
            // 初始化帧结构
            auto payload_length = payload_length_of(begin);
            // 确定帧长度
            auto frame_end      = begin + payload_length + 7;
            // 尚未接收完，退出
            if (frame_end > end) return {result_type_t::nothing};
            // crc 校验，限定返回值类型并准备查找下一个帧头
            // 找到下一个帧头
            while (true) {
                if (end - begin < 7)
                    return {result_type_t::nothing};
                if (destination_address_of(begin) == destination_address
                    && packet_type_of(begin) == packet_type)
                    break;
                ++begin;
            }
            return result;
        }
    };
}


#endif //PM1_SDK_PARSER_T_HPP
