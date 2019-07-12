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
                /** 二选一 */
                union {
                    word_t         bytes[sizeof(pack_with_data)];
                    pack_no_data   signal;
                    pack_with_data message;
                };
            };
            
            template<class iterator_t>
            result_t operator()(iterator_t &begin, iterator_t &end) const {
                // 找到一个帧头
                while (*begin != 0xfe)
                    if (++begin + sizeof(pack_no_data) > end)
                        return {result_type_t::nothing};
                
                // 初始化帧结构
                result_t result{result_type_t::nothing, {0xfe, begin[1]}};
                auto     frame_end = begin + (result.message.payload
                                              ? sizeof(pack_with_data)
                                              : sizeof(pack_no_data));
                
                if (frame_end > end) return result;
                
                auto checked = crc_check(begin + 1, frame_end);
                if (checked) {
                    result.type = result.message.payload
                                  ? result_type_t::message
                                  : result_type_t::signal;
                    
                    begin += 2;
                    auto ptr = result.bytes + 2;
                    while (begin < frame_end) *ptr++ = *begin++;
                } else {
                    result.type = result.message.payload
                                  ? result_type_t::message_failed
                                  : result_type_t::signal_failed;
                    ++begin;
                }
                
                for (; begin < end; ++begin)
                    if (*begin == 0xfe)
                        break;
                end = begin;
                return result;
            }
        };
    } // namespace can
} // namespace autolabor


#endif //PM1_SDK_PARSER_H
