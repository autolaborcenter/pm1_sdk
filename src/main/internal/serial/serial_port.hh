//
// Created by User on 2019/3/29.
//

#ifndef SERIAL_PORT_HH
#define SERIAL_PORT_HH


#include <string>
#include <atomic>
#include <mutex>

/** 串口 */
class serial_port final {
public:
    /**
     * 构造器
     */
    
    explicit serial_port(const std::string &name,
                         unsigned int baud_rate = 9600,
                         uint8_t check_period = 3,
                         uint8_t wait_period = 1,
                         size_t in_buffer_size = 0x100,
                         size_t out_buffer_size = 0x100);
    
    /**
     * 析构器
     */
    ~serial_port();
    
    /**
     * 发送
     */
    void send(const uint8_t *, size_t);
    
    /**
     * 读取
     * @return 实际读取的字节数
     */
    size_t read(uint8_t *, size_t);
    
    /**
     * 中断正在阻塞的读操作
     */
    void break_read() const;

private:
    #if   defined(_MSC_VER)
    using handler_t = void*;
    #elif defined(__GNUC__)
    using handler_t = int;
    mutable std::atomic_bool break_flag;
    #else
    #error unsupported platform
    #endif
    
    std::atomic<handler_t> handle;
    
    mutable std::mutex read_mutex;
};


#endif // SERIAL_PORT_HH
