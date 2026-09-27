#include "gm6020_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>


GM6020Hardware::GM6020Hardware(
    std::uint8_t id)
    : id_(id)
{
}


// ============================================================
// 设置控制量
// ============================================================

void GM6020Hardware::set_command(
    double command)
{
    command_ = command;
}


// ============================================================
// 获取角度
// ============================================================

double GM6020Hardware::angle() const
{
    return angle_;
}


// ============================================================
// 获取速度
// ============================================================

double GM6020Hardware::speed() const
{
    return speed_;
}


// ============================================================
// 保存 CAN 反馈
// ============================================================

void GM6020Hardware::store_status(
    const std::uint8_t* data,
    std::size_t length)
{
    if (data == nullptr || length != 8)
    {
        return;
    }

    std::memcpy(
        data_,
        data,
        8);
}


// ============================================================
// 解析 GM6020 CAN 反馈
// ============================================================

void GM6020Hardware::update_status()
{
    /*
     * GM6020 CAN feedback:
     *
     * byte 0-1 : encoder angle
     * byte 2-3 : velocity
     * byte 4-5 : current
     * byte 6   : temperature
     * byte 7   : unused
     */


    // ------------------------------------------------------------
    // 原始角度
    // ------------------------------------------------------------

    const int raw_angle =
        static_cast<int>(
            (static_cast<std::uint16_t>(data_[0]) << 8)
            | static_cast<std::uint16_t>(data_[1]));


    // ------------------------------------------------------------
    // 原始速度
    // ------------------------------------------------------------

    const std::int16_t raw_velocity =
        static_cast<std::int16_t>(
            (static_cast<std::uint16_t>(data_[2]) << 8)
            | static_cast<std::uint16_t>(data_[3]));


    // ============================================================
    // 角度展开
    // ============================================================
    //
    // GM6020 encoder:
    //
    // 0 ~ 8191
    //
    // 一圈之后会从 8191 跳回 0。
    //
    // 这里把这种跳变转换成连续角度。
    // ============================================================

    if (!angle_initialized_)
    {
        previous_raw_angle_ = raw_angle;

        accumulated_angle_ =
            static_cast<double>(raw_angle)
            / kRawAngleMax
            * 2.0
            * M_PI;

        angle_initialized_ = true;
    }
    else
    {
        int delta =
            raw_angle - previous_raw_angle_;


        // 正向跨过 0 点
        if (delta > kRawAngleMax / 2)
        {
            delta -= kRawAngleMax;
        }

        // 反向跨过 0 点
        else if (delta < -kRawAngleMax / 2)
        {
            delta += kRawAngleMax;
        }


        accumulated_angle_ +=
            static_cast<double>(delta)
            / kRawAngleMax
            * 2.0
            * M_PI;


        previous_raw_angle_ = raw_angle;
    }


    angle_ = accumulated_angle_;


    // ============================================================
    // 速度
    // ============================================================
    //
    // GM6020 feedback velocity:
    // rpm
    //
    // rpm -> rad/s
    // ============================================================

    speed_ =
        static_cast<double>(raw_velocity)
        / 60.0
        * 2.0
        * M_PI;
}


// ============================================================
// PID torque -> GM6020 raw current
// ============================================================

std::int16_t GM6020Hardware::generate_command() const
{
    // ------------------------------------------------------------
    // 限制 torque
    // ------------------------------------------------------------

    const double torque =
        std::clamp(
            command_,
            -kMaxTorque,
            kMaxTorque);


    // ------------------------------------------------------------
    // torque -> current
    // ------------------------------------------------------------

    const double current =
        torque
        / kTorqueConstant;


    // ------------------------------------------------------------
    // current -> GM6020 raw current
    // ------------------------------------------------------------

    const double raw_current =
        current
        / kCurrentMax
        * kRawCurrentMax;


    return static_cast<std::int16_t>(
        std::round(raw_current));
}


// ============================================================
// 写入 CAN 控制帧
// ============================================================

void GM6020Hardware::write_command_to_packet(
    CanPacket8& packet) const
{
    const std::int16_t raw_current =
        generate_command();


    /*
     * GM6020 ID 1~4:
     *
     * ID1 -> DATA[0:1]
     * ID2 -> DATA[2:3]
     * ID3 -> DATA[4:5]
     * ID4 -> DATA[6:7]
     */

    const std::size_t index =
        static_cast<std::size_t>(
            (id_ - 1) % 4);


    packet[index * 2] =
        static_cast<std::uint8_t>(
            (raw_current >> 8) & 0xFF);


    packet[index * 2 + 1] =
        static_cast<std::uint8_t>(
            raw_current & 0xFF);
}


// ============================================================
// 电机 ID
// ============================================================

std::uint8_t GM6020Hardware::id() const
{
    return id_;
}


// ============================================================
// GM6020 feedback CAN ID
// ============================================================
//
// GM6020:
// feedback ID = 0x204 + motor ID
//
// ID3:
// 0x204 + 3 = 0x207
// ============================================================

std::uint32_t GM6020Hardware::recv_id() const
{
    return 0x204 + id_;
}


// ============================================================
// GM6020 command CAN ID
// ============================================================
//
// ID1~4:
// 0x1FE
// ============================================================

std::uint32_t GM6020Hardware::send_id() const
{
    if (id_ <= 4)
    {
        return 0x1FE;
    }

    return 0x2FE;
}