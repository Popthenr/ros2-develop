#pragma once

#include <cstddef>
#include <cstdint>

#include "motor_input.hpp"
#include "can_packet.hpp"

class GM6020Hardware : public MotorInput
{
public:

    explicit GM6020Hardware(
        std::uint8_t id);

    void set_command(double command) override;

    double angle() const override;

    double speed() const override;

    void store_status(
        const std::uint8_t* data,
        std::size_t length);

    void update_status();

    std::int16_t generate_command() const;

    void write_command_to_packet(
        CanPacket8& packet) const;

    std::uint8_t id() const;

    std::uint32_t recv_id() const;

    std::uint32_t send_id() const;


private:

    std::uint8_t id_;

    double command_ = 0.0;

    // 连续角度，单位 rad
    double angle_ = 0.0;

    // 速度，单位 rad/s
    double speed_ = 0.0;

    std::uint8_t data_[8]{};


    // ============================================================
    // GM6020
    // ============================================================

    static constexpr int kRawAngleMax = 8192;

    static constexpr double kTorqueConstant = 0.741;

    static constexpr double kRawCurrentMax = 16384.0;

    static constexpr double kCurrentMax = 3.0;

    static constexpr double kMaxTorque =
        kTorqueConstant * kCurrentMax;


    // ============================================================
    // 角度展开
    // ============================================================

    int previous_raw_angle_ = 0;

    bool angle_initialized_ = false;

    double accumulated_angle_ = 0.0;
};