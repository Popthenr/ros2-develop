#include "gm6020_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

GM6020Hardware::GM6020Hardware(
    std::uint8_t id)
    : id_(id)
{
}

void GM6020Hardware::set_command(
    double command)
{
    command_ = command;
}

double GM6020Hardware::angle() const
{
    return angle_;
}

double GM6020Hardware::speed() const
{
    return speed_;
}

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

void GM6020Hardware::update_status()
{
   /*
     * GM6020 CAN feedback:
     *
     * byte 0-1 : angle
     * byte 2-3 : velocity
     * byte 4-5 : current
     * byte 6   : temperature
     * byte 7   : unused
     */

    const std::int16_t raw_angle =
        static_cast<std::int16_t>(
            (static_cast<std::uint16_t>(data_[0]) << 8)
            | static_cast<std::uint16_t>(data_[1]));

    const std::int16_t raw_velocity =
        static_cast<std::int16_t>(
            (static_cast<std::uint16_t>(data_[2]) << 8)
            | static_cast<std::uint16_t>(data_[3]));

    /*
     * GM6020 encoder:
     *
     * 0 ~ 8191
     * corresponds to
     * 0 ~ 2*pi rad
     */
    angle_ =
        static_cast<double>(raw_angle)
        / kRawAngleMax
        * 2.0
        * M_PI;

    /*
     * RMCS:
     *
     * raw velocity is rpm
     *
     * rpm -> rad/s
     */
    speed_ =
        static_cast<double>(raw_velocity)
        / 60.0
        * 2.0
        * M_PI;
}

std::int16_t GM6020Hardware::generate_command() const
{
    /*
     * PID output:
     *
     * command = torque (N*m)
     *
     * torque -> current
     *
     * RMCS:
     *
     * current =
     *     torque
     *     / torque_constant
     *     / current_max
     *     * raw_current_max
     */

    const double torque =
        std::clamp(
            command_,
            -kMaxTorque,
            kMaxTorque);

    const double current =
        torque
        / kTorqueConstant;

    const double raw_current =
        current
        / kCurrentMax
        * kRawCurrentMax;

    return static_cast<std::int16_t>(
        std::round(raw_current));
}

std::uint8_t GM6020Hardware::id() const
{
    return id_;
}

std::uint32_t GM6020Hardware::recv_id() const
{
    return 0x204 + id_;
}

std::uint32_t GM6020Hardware::send_id() const
{
    if (id_ <= 4)
    {
        return 0x1FE;
    }

    return 0x2FE;
}