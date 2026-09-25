#include <chrono>
#include <cmath>
#include <iostream>

#include "dr16.hpp"
#include "pid_controller.hpp"
#include "gm6020_hardware.hpp"
#include "lowpass_filter.hpp"
#include "can_interface.hpp"
#include "can_packet.hpp"

int main()
{
    // =========================
    // 1. 创建硬件对象
    // =========================

    hardware::Dr16 dr16;

    GM6020Hardware motor(1);

    CanInterface can;
    CanPacket8 tx_packet;

    if (!can.open("can0"))
    {
        std::cerr << "Failed to open CAN interface\n";
        return 1;
    }

    // =========================
    // 2. 创建两个 PID
    // =========================

    // 位置环
    PidController position_pid(
        1.0,
        0.0,
        0.0);

    // 速度环
    PidController speed_pid(
        0.5,
        1.0,
        0.001);

    // =========================
    // 3. 创建速度低通滤波器
    // =========================

    LowPassFilter speed_filter(0.1);

    // =========================
    // 4. 控制参数
    // =========================

    const double dt = 0.001;

    // 摇杆满量程对应的目标速度
    const double max_velocity = 10.0;

    // 目标角度
    double target_angle = 0.0;

    // =========================
    // 5. 控制循环
    // =========================

    for (int i = 0; i < 10000; ++i)
    {
        // -------------------------
        // DR16
        // -------------------------

        dr16.update_status();

        const double joystick_x =
            dr16.joystick_x();

        // 摇杆 → 目标速度
        const double target_velocity =
            joystick_x * max_velocity;

        // -------------------------
        // 目标速度 → 目标角度
        // -------------------------

        target_angle +=
            target_velocity * dt;

        // -------------------------
        // 获取电机反馈
        // -------------------------
        
        std::uint32_t can_id;

        CanPacket8 rx_packet;

        if (can.receive(can_id, rx_packet))
        {
           if (can_id == motor.recv_id())
           {
               motor.store_status(
                   rx_packet.data.data(),
                   rx_packet.data.size());

               motor.update_status();
            }
        }

        const double actual_angle =
            motor.angle();

        const double actual_speed =
            motor.speed();

        // -------------------------
        // 速度低通滤波
        // -------------------------

        const double filtered_speed =
            speed_filter.update(actual_speed);

        // -------------------------
        // 位置环
        // -------------------------

        const double target_speed =
            position_pid.update(
                target_angle,
                actual_angle,
                dt);

        // -------------------------
        // 速度环
        // -------------------------

        const double torque =
            speed_pid.update(
                target_speed,
                filtered_speed,
                dt);

        // -------------------------
        // 给电机
        // -------------------------

        motor.set_command(torque);

        const auto raw_current =
            motor.generate_command();

        tx_packet[0] =
           static_cast<std::uint8_t>(
                (raw_current >> 8) & 0xFF);

        tx_packet[1] =
            static_cast<std::uint8_t>(
                raw_current & 0xFF);

        can.send(
            motor.send_id(),
            tx_packet);

        // -------------------------
        // 输出观察
        // -------------------------

        if (i % 100 == 0)
        {
            std::cout
                << "joystick = "
                << joystick_x
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
    }

    return 0;
}