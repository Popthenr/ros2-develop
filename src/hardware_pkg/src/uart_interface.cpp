#include "uart_interface.hpp"

#include <cerrno>
#include <cstring>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <asm/termbits.h>

UartInterface::UartInterface()
{
}

UartInterface::~UartInterface()
{
    close();
}

bool UartInterface::open(
    const char* device,
    int baudrate)
{
    fd_ = ::open(
        device,
        O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fd_ < 0)
    {
        std::cerr
            << "Failed to open UART "
            << device
            << ": "
            << std::strerror(errno)
            << '\n';

        return false;
    }

    struct termios2 tty{};

    if (ioctl(
            fd_,
            TCGETS2,
            &tty) != 0)
    {
        std::cerr
            << "Failed to get UART settings: "
            << std::strerror(errno)
            << '\n';

        close();
        return false;
    }

    tty.c_cflag &= ~CBAUD;
    tty.c_cflag |= BOTHER;

    tty.c_ispeed = baudrate;
    tty.c_ospeed = baudrate;

    // 8 data bits
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    // 偶校验
    tty.c_cflag |= PARENB;
    tty.c_cflag &= ~PARODD;

    // 1 stop bit
    tty.c_cflag &= ~CSTOPB;

    // 不使用硬件流控
    tty.c_cflag &= ~CRTSCTS;

    // 接收使能 + 本地连接
    tty.c_cflag |= CREAD | CLOCAL;

    // 原始模式
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;

    if (ioctl(
            fd_,
            TCSETS2,
            &tty) != 0)
    {
        std::cerr
            << "Failed to set UART settings: "
            << std::strerror(errno)
            << '\n';

        close();
        return false;
    }

    return true;
}

void UartInterface::close()
{
    if (fd_ >= 0)
    {
        ::close(fd_);
        fd_ = -1;
    }
}

bool UartInterface::read(
    std::byte* data,
    std::size_t length)
{
    if (fd_ < 0 ||
        data == nullptr ||
        length == 0)
    {
        return false;
    }

    std::byte temp[64]{};

    const ssize_t result =
        ::read(
            fd_,
            temp,
            sizeof(temp));

    if (result <= 0)
    {
        return false;
    }

    const std::size_t received =
        static_cast<std::size_t>(result);

    if (buffer_size_ + received >
        buffer_.size())
    {
        buffer_size_ = 0;
        return false;
    }

    for (std::size_t i = 0;
         i < received;
         ++i)
    {
        buffer_[buffer_size_ + i] =
            temp[i];
    }

    buffer_size_ += received;

    if (buffer_size_ < length)
    {
        return false;
    }

    for (std::size_t i = 0;
         i < length;
         ++i)
    {
        data[i] = buffer_[i];
    }

    const std::size_t remaining =
        buffer_size_ - length;

    for (std::size_t i = 0;
         i < remaining;
         ++i)
    {
        buffer_[i] =
            buffer_[length + i];
    }

    buffer_size_ = remaining;

    return true;
}