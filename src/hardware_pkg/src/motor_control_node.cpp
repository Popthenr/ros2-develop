#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <thread>

#include <librmcs/board/c_board.hpp>
#include <librmcs/data/datas.hpp>

#include "hardware/device/dr16.hpp"

#include "gm6020_hardware.hpp"
#include "lowpass_filter.hpp"
#include "pid_controller.hpp"


using namespace std::chrono_literals;


class MotorControlSystem
    : public librmcs::board::CBoard::Callback
{
public:

    MotorControlSystem()
        : dr16_(),
          motor_(1),
          position_pid_(8.0, 0.0, 0.0),
          speed_pid_(0.02, 0.0, 0.0),
          speed_filter_(0.1)
    {
    }


    // ============================================================
    // 启动 C Board
    // ============================================================

    void start()
    {
        board_ =
            std::make_unique<
                librmcs::board::CBoard>(*this);


        std::cout
            << "Motor control system started."
            << '\n';

        std::cout
            << "GM6020 ID = "
            << static_cast<int>(motor_.id())
            << '\n';

        std::cout
            << "GM6020 RX CAN ID = 0x"
            << std::hex
            << motor_.recv_id()
            << std::dec
            << '\n';

        std::cout
            << "GM6020 TX CAN ID = 0x"
            << std::hex
            << motor_.send_id()
            << std::dec
            << '\n';
    }


    // ============================================================
    // 1 kHz 控制循环
    // ============================================================

    void update()
    {
        constexpr double dt = 0.001;


        // ========================================================
        // 1. 更新 DR16
        // ========================================================

        dr16_.update_status();

        const bool dr16_valid =
            dr16_.valid();


        // ========================================================
        // 2. 读取右摇杆
        // ========================================================

        /*
         * RMCS:
         *
         * physical right  -> joystick_y < 0
         * physical left   -> joystick_y > 0
         *
         * 所以反号：
         *
         * physical right  -> positive
         * physical left   -> negative
         */

        const double joystick_y =
            dr16_.joystick_right().y();

        const double joystick_horizontal =
            -joystick_y;

        constexpr double kMaxTargetSpeed =
            10.0;

        // 摇杆死区
        constexpr double kJoystickDeadzone =
            0.05;

        double target_velocity = 0.0;

        if (dr16_valid)
        {
        if (std::abs(joystick_horizontal) >
            kJoystickDeadzone)
            {
              target_velocity =
                    joystick_horizontal
                    * kMaxTargetSpeed;
            }
        else
            {
            target_velocity = 0.0;
            }
        }

        if (target_angle_initialized_)
        {
        target_angle_ +=
            target_velocity * dt;
        }


        // ========================================================
        // 5. 读取 GM6020 当前状态
        // ========================================================

        const double actual_angle =
            motor_.angle();


        const double actual_speed =
            speed_filter_.update(
                motor_.speed());


        // ========================================================
        // 6. 外环：位置 PID
        // ========================================================

        double target_speed = 0.0;


        if (target_angle_initialized_)
        {
            target_speed =
                position_pid_.update(
                    target_angle_,
                    actual_angle,
                    dt);
        }


        // ========================================================
        // 7. 内环：速度 PID
        // ========================================================

        double torque = 0.3;


        if (target_angle_initialized_)
        {
            torque =
                speed_pid_.update(
                    target_speed,
                    actual_speed,
                    dt);
        }


        // ========================================================
        // 8. 限制 torque
        // ========================================================

        constexpr double kMaxTorque =
            2.0;


        if (torque > kMaxTorque)
        {
            torque = kMaxTorque;
        }

        if (torque < -kMaxTorque)
        {
            torque = -kMaxTorque;
        }


        // ========================================================
        // 9. CAN 发送
        // ========================================================
        //
        // 控制算法：
        // 1 kHz
        //
        // CAN：
        // 100 Hz
        //
        // 防止 CBoard transmit buffer 被连续占满。
        // ========================================================

        static int send_count = 0;

        ++send_count;


        if (send_count >= 10)
        {
            send_count = 0;


            motor_.set_command(
                torque);


            send_motor_command();
        }


        // ========================================================
        // 10. 调试信息
        // ========================================================

        static int debug_count = 0;

        ++debug_count;


        if (debug_count >= 100)
        {
            debug_count = 0;


            std::cout
                << "DR16="
                << dr16_valid

                << " | joystick="
                << joystick_horizontal

                << " | target_vel="
                << target_velocity

                << " | target_angle="
                << target_angle_

                << " | actual_angle="
                << actual_angle

                << " | actual_speed="
                << actual_speed

                << " | target_speed="
                << target_speed

                << " | torque="
                << torque

                << '\n';
        }
    }


private:

    // ============================================================
    // DR16 UART 接收
    // ============================================================

    void uart_receive_callback(
        const Spec::Uart& uart,
        const View::Uart& data) override
    {
        if (uart != Spec::kUarts.kDbus)
        {
            return;
        }


        dr16_.store_status(
            data.uart_data.data(),
            data.uart_data.size());
    }


    // ============================================================
    // CAN 接收
    // ============================================================

    void can_receive_callback(
        const Spec::Can& can,
        const View::Can& data) override
    {
        // 只处理 CAN1
        if (can != Spec::kCans.kCan1)
        {
            return;
        }

        std::cout
            << "CAN RX ID = 0x"
            << std::hex
            << data.can_id
            << std::dec
            << " | len = "
            << data.can_data.size()
            << '\n';


        // ========================================================
        // 只接受 GM6020 ID3
        //
        // GM6020 ID3:
        //
        // 0x204 + 3 = 0x207
        //
        // 其他 CAN ID，例如 0x203：
        // 直接忽略。
        // ========================================================

        if (data.can_id != motor_.recv_id())
        {
            return;
        }


        // GM6020 feedback 必须 8 bytes
        if (data.can_data.size() != 8)
        {
            return;
        }


        std::array<std::uint8_t, 8> raw_data{};


        for (std::size_t i = 0; i < 8; ++i)
        {
            raw_data[i] =
                static_cast<std::uint8_t>(
                    data.can_data[i]);
        }


        // 保存反馈
        motor_.store_status(
            raw_data.data(),
            raw_data.size());


        // 解析反馈
        motor_.update_status();


        // ========================================================
        // 第一次收到 GM6020 反馈
        // ========================================================

        if (!target_angle_initialized_)
        {
            target_angle_ =
                motor_.angle();


            target_angle_initialized_ =
                true;


            std::cout
                << "GM6020 feedback detected."
                << " CAN ID = 0x"
                << std::hex
                << data.can_id
                << std::dec
                << " | initial angle = "
                << motor_.angle()
                << '\n';
        }
    }


    // ============================================================
    // 发送 GM6020 控制帧
    // ============================================================

    void send_motor_command()
    {
        // 必须初始化
        CanPacket8 packet{};


        /*
         * GM6020 ID3:
         *
         * CAN ID = 0x1FE
         *
         * DATA[4] = current high byte
         * DATA[5] = current low byte
         */

        motor_.write_command_to_packet(
            packet);


        std::array<std::byte, 8> can_data{};


        for (std::size_t i = 0; i < 8; ++i)
        {
            can_data[i] =
                static_cast<std::byte>(
                    packet[i]);
        }


        librmcs::data::CanDataView data{
            motor_.send_id(),
            std::span<const std::byte>(
                can_data.data(),
                can_data.size())
        };


        board_->start_transmit()
            .can_transmit(
                Spec::kCans.kCan1,
                data);
    }


private:

    // ============================================================
    // DR16
    // ============================================================

    rmcs_core::hardware::device::Dr16 dr16_;


    // ============================================================
    // GM6020 ID3
    // ============================================================

    GM6020Hardware motor_;


    // ============================================================
    // 外环：位置 PID
    // ============================================================

    PidController position_pid_;


    // ============================================================
    // 内环：速度 PID
    // ============================================================

    PidController speed_pid_;


    // ============================================================
    // 速度低通滤波
    // ============================================================

    LowPassFilter speed_filter_;


    // ============================================================
    // 目标角度
    // ============================================================

    double target_angle_ = 0.0;

    bool target_angle_initialized_ = false;


    // ============================================================
    // C Board
    // ============================================================

    std::unique_ptr<
        librmcs::board::CBoard>
        board_;
};


int main()
{
    MotorControlSystem system;


    system.start();


    while (true)
    {
        system.update();

        std::this_thread::sleep_for(
            1ms);
    }


    return 0;
}