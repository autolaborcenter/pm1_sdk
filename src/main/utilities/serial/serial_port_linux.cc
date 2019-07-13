//
// Created by User on 2019/3/29.
//

#include "serial_port.hh"

#ifdef __GNUC__

#include <vector>
#include <thread>
#include "macros.h"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>

#define TRY(OPERATION) if(!OPERATION) THROW(#OPERATION, std::strerror(errno))

inline int trans_baud(int number);

enum class read_state_t {
    check, read, wait
};

serial_port::serial_port(
    const std::string &name,
    unsigned int baud_rate,
    uint8_t check_period,
    uint8_t, size_t, size_t
) : break_flag(false) {
    
    handle = open(name.c_str(), O_RDWR | O_NOCTTY);
    
    if (handle == -1)
        THROW("open(...)", std::strerror(errno));
    
    // 设置端口设定
    termios options{};
    TRY(!tcgetattr(handle, &options));
    cfsetispeed(&options, trans_baud(baud_rate));
    cfsetospeed(&options, trans_baud(baud_rate));
    
    // 8N1, no flow control
    options.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    options.c_cflag |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines
    options.c_cflag |= CS8;
    
    options.c_lflag =
    options.c_iflag =
    options.c_oflag = 0;
    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = check_period;
    
    TRY(!tcsetattr(handle, TCSANOW, &options));
}

serial_port::~serial_port() {
    auto temp = handle.exchange(0);
    if (!temp) return;
    break_read();
    close(temp);
}

void serial_port::send(const uint8_t *buffer, size_t size) {
    if (size > 0) TRY(size == write(handle, buffer, size));
}

size_t serial_port::read(uint8_t *buffer, size_t size) {
    weak_lock_guard lock(read_mutex);
    if (!lock) return 0;
    
    while (true) {
        auto temp = ::read(handle, buffer, size);
        if (temp > 0 || break_flag) return temp;
    }
}

void serial_port::break_read() const {
    weak_lock_guard lock(read_mutex);
    break_flag = true;
    while (!lock.retry())
        std::this_thread::yield();
    break_flag = false;
}

int trans_baud(int number) {
    switch (number) {
        case 0:
            return B0;
        case 50:
            return B50;
        case 75:
            return B75;
        case 110:
            return B110;
        case 134:
            return B134;
        case 150:
            return B150;
        case 200:
            return B200;
        case 300:
            return B300;
        case 600:
            return B600;
        case 1200:
            return B1200;
        case 1800:
            return B1800;
        case 2400:
            return B2400;
        case 4800:
            return B4800;
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 460800:
            return B460800;
        case 500000:
            return B500000;
        case 576000:
            return B576000;
        case 921600:
            return B921600;
        case 1000000:
            return B1000000;
        case 1152000:
            return B1152000;
        case 1500000:
            return B1500000;
        case 2000000:
            return B2000000;
        case 2500000:
            return B2500000;
        case 3000000:
            return B3000000;
        case 3500000:
            return B3500000;
        case 4000000:
            return B4000000;
        default:
            THROW("set baud rate", "unsupported value");
    }
}

#endif
