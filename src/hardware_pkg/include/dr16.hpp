#pragma once

#include <atomic>
#include <bit>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace hardware
{

class Dr16
{
public:
    Dr16() = default;

    // 接收一帧 DR16 原始数据
    void store_status(const std::byte* uart_data,
                      std::size_t uart_data_length)
    {
        // DR16 一帧数据长度为 18 字节
        if (uart_data_length != 18)
            return;

        // 前 6 字节暂时跳过
        uart_data += 6;

        // 读取中间 8 字节
        uint64_t part1{};
        std::memcpy(&part1, uart_data, 8);

        data_part1_.store(part1, std::memory_order_relaxed);

        last_received_at_ = Clock::now();
        valid_ = true;
    }

    // 更新摇杆状态
    void update_status()
    {
        const auto now = Clock::now();

        refresh_validity(now);

        if (!valid_)
            return;

        const auto part1 =
            std::bit_cast<Dr16DataPart1>(
                data_part1_.load(std::memory_order_relaxed));

        /*
         * DR16 原始摇杆通道：
         *
         * channel0 → 右摇杆 Y
         * channel1 → 右摇杆 X
         * channel2 → 左摇杆 Y
         * channel3 → 左摇杆 X
         *
         * 我们这里只使用 channel1：
         * 右摇杆 X
         */

        joystick_x_ =
            channel_to_double(
                static_cast<int32_t>(part1.joystick_channel1));
    }

    // 获取右摇杆 X
    // 范围：-1.0 ~ +1.0
    double joystick_x() const
    {
        return joystick_x_;
    }

    // DR16 数据是否有效
    bool valid() const noexcept
    {
        return valid_;
    }

    // 是否启用超时保护
    void set_timeout_enabled(bool enabled)
    {
        timeout_enabled_ = enabled;
    }

private:

    // 将 DR16 原始通道值转换成 -1 ~ +1
    static double channel_to_double(int32_t value)
    {
        /*
         * DR16 摇杆中心值约为 1024
         *
         * 原始值：
         *     364 ~ 1684
         *
         * 转换后：
         *     -1 ~ +1
         */

        value -= 1024;

        if (-660 <= value && value <= 660)
            return value / 660.0;

        return 0.0;
    }

    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    static constexpr auto kFreshTimeout =
        std::chrono::milliseconds(500);

    void refresh_validity(const TimePoint now)
    {
        if (!timeout_enabled_ ||
            !valid_ ||
            now - last_received_at_ <= kFreshTimeout)
        {
            return;
        }

        joystick_x_ = 0.0;
        valid_ = false;
    }

    /*
     * DR16 数据的第 6~13 字节：
     *
     * 4 个摇杆，每个 11 bit
     * 2 个拨杆，每个 2 bit
     *
     * 总共：
     *
     * 11 × 4 + 2 × 2 = 48 bit
     * = 6 字节
     *
     * 但我们这里为了方便直接读取原 RMCS
     * 对齐后的 8 字节结构。
     */
    struct [[gnu::packed]] Dr16DataPart1
    {
        uint64_t joystick_channel0 : 11;
        uint64_t joystick_channel1 : 11;
        uint64_t joystick_channel2 : 11;
        uint64_t joystick_channel3 : 11;

        uint64_t switch_right : 2;
        uint64_t switch_left  : 2;

        uint64_t padding : 16;
    };

    static_assert(sizeof(Dr16DataPart1) == 8);

    std::atomic<uint64_t> data_part1_{
        std::bit_cast<uint64_t>(
            Dr16DataPart1{
                .joystick_channel0 = 1024,
                .joystick_channel1 = 1024,
                .joystick_channel2 = 1024,
                .joystick_channel3 = 1024,
                .switch_right = 0,
                .switch_left = 0,
                .padding = 0
            })
    };

    double joystick_x_ = 0.0;

    TimePoint last_received_at_ =
        TimePoint::min();

    bool valid_ = false;

    bool timeout_enabled_ = true;
};

}  // namespace hardware