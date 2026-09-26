#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>

#include "hardware/device/dr16.hpp"
#include "pid_controller.hpp"
#include "gm6020_hardware.hpp"
#include "lowpass_filter.hpp"
#include "can_interface.hpp"
#include "can_packet.hpp"
#include "uart_interface.hpp"

int main()
{
    // =========================
    // 1. 创建各个硬件/控制模块
    // =========================

    rmcs_core::hardware::device::Dr16 dr16;

    GM6020Hardware motor(1);

    CanInterface can;

    UartInterface uart;

    PidController position_pid(
        1.0,
        0.0,
        0.0);

    PidController speed_pid(
        0.5,
        1.0,
        0.001);

    LowPassFilter speed_filter(0.1);


    // =========================
    // 2. 控制参数
    // =========================

    const double dt = 0.001;

    // 摇杆满量程对应的目标速度
    // 单位：rad/s
    const double max_velocity = 10.0;

    // 目标角度
    double target_angle = 0.0;


    // =========================
    // 3. 打开 CAN
    // =========================

    if (!can.open("can0"))
    {
        std::cerr
            << "Failed to open CAN interface\n";

        return 1;
    }

    if (!uart.open(
        "/dev/ttyUSB0",
        100000))
{
    std::cerr
        << "Failed to open DR16 UART\n";

    return 1;
}


    // =========================
    // 4. 控制循环
    // =========================

    while (true)
    {
        // -------------------------
        // 4.1 更新 DR16 状态
        // -------------------------

        std::byte dr16_data[18]{};

        if (uart.read(
                dr16_data,
                sizeof(dr16_data)))
        {
            dr16.store_status(
                dr16_data,
                sizeof(dr16_data));
        }

        dr16.update_status();

        const double joystick_x =
            dr16.joystick_right().x();


        // -------------------------
        // 4.2 摇杆 → 目标速度
        // -------------------------

        const double target_velocity =
            joystick_x * max_velocity;


        // -------------------------
        // 4.3 目标速度 → 目标角度
        // -------------------------

        target_angle +=
            target_velocity * dt;


        // -------------------------
        // 4.4 接收 GM6020 CAN 数据
        // -------------------------

        std::uint32_t can_id;

        CanPacket8 rx_packet;

        if (can.receive(
                can_id,
                rx_packet))
        {
            if (can_id == motor.recv_id())
            {
                motor.store_status(
                    rx_packet.data.data(),
                    rx_packet.data.size());

                motor.update_status();
            }
        }


        // -------------------------
        // 4.5 获取电机反馈
        // -------------------------

        const double actual_angle =
            motor.angle();

        const double actual_speed =
            motor.speed();


        // -------------------------
        // 4.6 速度低通滤波
        // -------------------------

        const double filtered_speed =
            speed_filter.update(
                actual_speed);


        // -------------------------
        // 4.7 位置环
        //
        // 目标角度
        //     ↓
        // Position PID
        //     ↓
        // 目标速度
        // -------------------------

        const double target_speed =
            position_pid.update(
                target_angle,
                actual_angle,
                dt);


        // -------------------------
        // 4.8 速度环
        //
        // 目标速度
        //     ↓
        // Speed PID
        //     ↓
        // 扭矩
        // -------------------------

        const double torque =
            speed_pid.update(
                target_speed,
                filtered_speed,
                dt);


        // -------------------------
        // 4.9 将扭矩交给 GM6020
        // -------------------------

        motor.set_command(
            torque);


        // -------------------------
        // 4.10 扭矩 → GM6020 原始电流
        // -------------------------

        CanPacket8 tx_packet;

        motor.write_command_to_packet(
            tx_packet);

        // -------------------------
        // 4.12 发送 CAN
        // -------------------------

        can.send(
            motor.send_id(),
            tx_packet);


        // -------------------------
        // 4.13 调试输出
        // -------------------------

        static int print_count = 0;

        ++print_count;

        if (print_count >= 100)
        {
            print_count = 0;

            std::cout
                << "joystick = "
                << joystick_x

                << " | target_velocity = "
                << target_velocity

                << " | target_angle = "
                << target_angle

                << " | actual_angle = "
                << actual_angle

                << " | target_speed = "
                << target_speed

                << " | speed = "
                << filtered_speed

                << " | torque = "
                << torque

                << '\n';
        }


        // -------------------------
        // 4.14 等待 1 ms
        // -------------------------

        std::this_thread::sleep_for(
            std::chrono::milliseconds(1));
    }


    return 0;
}