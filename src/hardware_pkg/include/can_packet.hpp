#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

struct CanPacket8
{
    std::array<std::uint8_t, 8> data{};

    CanPacket8() = default;

    CanPacket8(
        const std::uint8_t* input,
        std::size_t length)
    {
        if (input == nullptr || length != 8)
        {
            return;
        }

        std::memcpy(
            data.data(),
            input,
            8);
    }

    std::uint8_t& operator[](std::size_t index)
    {
        return data[index];
    }

    const std::uint8_t& operator[](
        std::size_t index) const
    {
        return data[index];
    }
};