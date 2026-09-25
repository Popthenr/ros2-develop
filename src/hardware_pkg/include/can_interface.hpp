#pragma once

#include <cstdint>

#include "can_packet.hpp"

class CanInterface
{
public:
    CanInterface();
    ~CanInterface();

    // 打开 CAN 接口
    bool open(const char* interface_name);

    // 关闭 CAN 接口
    void close();

    // 发送 CAN 数据
    bool send(
        std::uint32_t can_id,
        const CanPacket8& packet);

    // 接收 CAN 数据
    bool receive(
        std::uint32_t& can_id,
        CanPacket8& packet);

private:
    // Linux CAN socket
    int socket_fd_ = -1;
};