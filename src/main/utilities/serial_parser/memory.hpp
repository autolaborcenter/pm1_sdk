//
// Created by ydrml on 2019/7/13.
//

#ifndef PM1_SDK_MEMORY_HPP
#define PM1_SDK_MEMORY_HPP

/**
 * 内存解释器
 * @return 视作字节数组的内存起点
 */
template<class t>
constexpr inline unsigned char *bytes_begin(t &data) { return (unsigned char *) (&data); }

/**
 * 内存解释器
 * @return 视作字节数组的内存终点
 */
template<class t>
constexpr inline unsigned char *bytes_end(t &data) { return bytes_begin(data) + sizeof(t); }


#endif //PM1_SDK_MEMORY_HPP
