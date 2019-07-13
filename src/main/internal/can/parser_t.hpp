//
// Created by ydrml on 2019/2/26.
//

#ifndef PM1_SDK_PARSER_H
#define PM1_SDK_PARSER_H


#include "info.h"

namespace autolabor {
    namespace can {
        /** 解析器 */
        struct parser_t {
            /** 字类型（作为解析器） */
            using word_t = uint8_t;
            
            /** 结果类型 */
            enum class result_type_t : uint8_t {
                /** 无 */
                    nothing,
                /** 信号（校验错误） */
                    signal_failed,
                /** 消息（校验错误） */
                    message_failed,
                /** 信号 */
                    signal,
                /** 消息 */
                    message
            };
            
            /** 结果结构体 */
            struct result_t {
                /** 结果类型 */
                result_type_t type;
                /** 三选一 */
                union {
                    word_t         bytes[sizeof(pack_with_data)];
                    pack_no_data   signal;
                    pack_with_data message;
                };
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
                while (*begin != 0xfe)
                    if (++begin + sizeof(pack_no_data) > end)
                        return {result_type_t::nothing};
                // 初始化帧结构
                result_t result{result_type_t::nothing, {0xfe, begin[1]}};
                // 确定帧长度
                auto     frame_end = begin + (result.message.payload
                                              ? sizeof(pack_with_data)
                                              : sizeof(pack_no_data));
                // 尚未接收完，退出
                if (frame_end > end) return result;
                // crc 校验，限定返回值类型并准备查找下一个帧头
                if (crc_check(begin + 1, frame_end)) {
                    // 成功，拷贝到返回值
                    std::copy(begin + 2, frame_end, result.bytes + 2);
                    result.type = result.message.payload
                                  ? result_type_t::message
                                  : result_type_t::signal;
                    begin = frame_end;
                } else {
                    result.type = result.message.payload
                                  ? result_type_t::message_failed
                                  : result_type_t::signal_failed;
                    ++begin;
                }
                // 找到下一个帧头
                while (begin < end && *begin != 0xfe) ++begin;
                end = begin;
                return result;
            }
        };
    } // namespace can
} // namespace autolabor


#endif //PM1_SDK_PARSER_H
