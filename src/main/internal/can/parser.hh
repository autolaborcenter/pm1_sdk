//
// Created by ydrml on 2019/2/26.
//

#ifndef PM1_SDK_PARSER_H
#define PM1_SDK_PARSER_H


#include "info.h"

namespace autolabor {
    namespace can {
        /** 解析器 */
        class parser {
        public:
            /** 结果类型 */
            enum class result_type : uint8_t {
                /** 无 */
                    nothing,
                /* 内部错误 **/
                    failed,
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
            struct result {
                /** 结果类型 */
                result_type type;
                /** 二选一 */
                union {
                    union_no_data   signal;
                    union_with_data message;
                };
            };
            
            /** 默认构造 */
            parser() = default;
            
            /** 实现复制 */
            parser(const parser &others);
            
            /** 禁止移动 */
            parser(parser &&others) = delete;
            
            /** 逐字节解析 */
            result operator()(uint8_t byte);
        
        private:
            enum class state_type : uint8_t {
                origin,
                determine,
                signal,
                message,
                ending
            };
            
            struct state_t {
                uint8_t value;
                
                state_type state();
            } state{0};
            
            union {
                uint8_t bytes[sizeof(pack_with_data)]{0xfe};
                
                union_no_data   sgn_buffer;
                union_with_data msg_buffer;
            };
        };
    }
}


#endif //PM1_SDK_PARSER_H
