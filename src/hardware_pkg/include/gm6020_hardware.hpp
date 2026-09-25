#pragma once

#include <cstddef>
#include <cstdint>

#include "motor_input.hpp"

class GM6020Hardware : public MotorInput
{
public:
    explicit GM6020Hardware(
        std::uint8_t id);

    //Motorinput接口
    void set_command(double command) override;

    double angle() const override;

    double speed() const override;

    // 保存收到的 CAN 数据
    void store_status(
        const std::uint8_t* data,
        std::size_t length);

    // 解析保存的反馈数据
    void update_status();

    // 生成发送给电机的原始电流指令
    std::int16_t generate_command() const;

    std::uint8_t id() const;

    std::uint32_t recv_id() const;

    std::uint32_t send_id() const;

private:
    std::uint8_t id_;

    // PID 给出的控制量
    double command_ = 0.0;

    // 电机反馈
    double angle_ = 0.0;
    double speed_ = 0.0;

    // 原始反馈数据
    std::uint8_t data_[8]{};

    // GM6020 编码器
    static constexpr int kRawAngleMax = 8192;

    // GM6020 扭矩参数
    static constexpr double kTorqueConstant = 0.741;

    // GM6020 原始电流最大值
    static constexpr double kRawCurrentMax = 16384.0;

    // GM6020 最大电流
    static constexpr double kCurrentMax = 3.0;

    // 最大扭矩
    static constexpr double kMaxTorque =
        kTorqueConstant * kCurrentMax;
};