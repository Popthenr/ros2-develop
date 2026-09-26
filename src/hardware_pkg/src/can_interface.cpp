#include "can_interface.hpp"

#include <cstring>
#include <fcntl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <net/if.h>

#include <sys/ioctl.h>
#include <sys/socket.h>

#include <unistd.h>

CanInterface::CanInterface()
{
}

CanInterface::~CanInterface()
{
    close();
}

bool CanInterface::open(
    const char* interface_name)
{
    socket_fd_ =
        socket(
            PF_CAN,
            SOCK_RAW,
            CAN_RAW);

    if (socket_fd_ < 0)
    {
        return false;
    }

    struct ifreq ifr{};

    std::strncpy(
        ifr.ifr_name,
        interface_name,
        IFNAMSIZ - 1);

    if (ioctl(
            socket_fd_,
            SIOCGIFINDEX,
            &ifr) < 0)
    {
        close();
        return false;
    }

    struct sockaddr_can address{};

    address.can_family = AF_CAN;
    address.can_ifindex = ifr.ifr_ifindex;

    if (bind(
            socket_fd_,
            reinterpret_cast<struct sockaddr*>(&address),
            sizeof(address)) < 0)
    {
        close();
        return false;
    }

    int flags = fcntl(
        socket_fd_,
        F_GETFL,
        0);

    if (flags < 0)
    {
        close();
        return false;
    }

    if (fcntl(
            socket_fd_,
            F_SETFL,
            flags | O_NONBLOCK) < 0)
    {
        close();
        return false;
    }

    return true;
}

void CanInterface::close()
{
    if (socket_fd_ >= 0)
    {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
}

bool CanInterface::send(
    std::uint32_t can_id,
    const CanPacket8& packet)
{
    if (socket_fd_ < 0)
    {
        return false;
    }

    struct can_frame frame{};

    frame.can_id = can_id;
    frame.can_dlc = 8;

    std::memcpy(
        frame.data,
        packet.data.data(),
        8);

    const auto result =
        ::send(
            socket_fd_,
            &frame,
            sizeof(frame),
            0);

    return result == sizeof(frame);
}

bool CanInterface::receive(
    std::uint32_t& can_id,
    CanPacket8& packet)
{
    if (socket_fd_ < 0)
    {
        return false;
    }

    struct can_frame frame{};

    const auto result =
        ::recv(
            socket_fd_,
            &frame,
            sizeof(frame),
            0);

    if (result != sizeof(frame))
    {
        return false;
    }

    can_id = frame.can_id;

    std::memcpy(
        packet.data.data(),
        frame.data,
        8);

    return true;
}