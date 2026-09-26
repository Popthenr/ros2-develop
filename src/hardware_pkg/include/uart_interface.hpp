#pragma once

#include <cstddef>
#include <array>

class UartInterface
{
public:
    UartInterface();
    ~UartInterface();

    bool open(
        const char* device,
        int baudrate);

    void close();

    bool read(
        std::byte* data,
        std::size_t length);

private:
    int fd_ = -1;

    std::array<std::byte, 256> buffer_{};

    std::size_t buffer_size_ = 0;
};