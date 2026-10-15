#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <thread>

#include <librmcs/board/c_board.hpp>
#include <librmcs/data/datas.hpp>

#include "hardware/device/dr16.hpp"

#include "lowpass_filter.hpp"
#include "pid_controller.hpp"


using namespace std::chrono_literals;


// ================================================================
// M2006 三电机控制系统
//
// 电机分配：
//
// M2006 ID1 -> Pitch 电机 1
// M2006 ID2 -> Pitch 电机 2
// M2006 ID3 -> Yaw   电机
//
// CAN：
//
// ID1 feedback -> 0x201
// ID2 feedback -> 0x202
// ID3 feedback -> 0x203
//
// 三个电机控制帧：
//
// TX -> 0x200
//
// DATA[0~1] -> ID1
// DATA[2~3] -> ID2
// DATA[4~5] -> ID3
// DATA[6~7] -> ID4
//
// 当前阶段：
//
// 1. 一个 motor_control_node 控制三个 M2006
// 2. Pitch 使用两个 M2006
// 3. Yaw 使用一个 M2006
// 4. 三个电机都有独立反馈
// 5. 三个电机都有独立速度内环
// 6. Pitch 两个电机共享 Pitch 目标角度
// 7. Yaw 使用独立目标角度
//
// 后续再优化：
//
// 1. Pitch 双电机同步
// 2. Pitch PID 参数
// 3. Yaw PID 参数
// 4. Pitch / Yaw 解耦
// ================================================================


// ================================================================
// M2006 电机反馈数据
// ================================================================

struct M2006Motor
{
    // 电机 ID
    int id = 0;

    // CAN 接收 ID
    int recv_id = 0;

    // 编码器原始值
    std::uint16_t encoder = 0;

    // 上一次编码器值
    std::uint16_t last_encoder = 0;

    // 电机转速 RPM
    double speed = 0.0;

    // 实际转矩电流
    double current = 0.0;

    // 温度
    std::uint8_t temperature = 0;

    // 连续角度
    //
    // 单位：degree
    //
    // 不是 0~360，而是可以一直累积：
    //
    // 0
    // 360
    // 720
    // ...
    double angle = 0.0;

    // 上一次连续角度
    double last_angle = 0.0;

    // 是否已经收到过反馈
    bool initialized = false;

    // 当前控制电流
    //
    // C610：
    // -10000 ~ 10000
    int command_current = 0;
};


// ================================================================
// 三电机控制系统
// ================================================================

class MotorControlSystem
    : public librmcs::board::CBoard::Callback
{
public:

    MotorControlSystem()
        : dr16_(),

          // ======================================================
          // 三个 M2006
          // ======================================================

          motor_1_{
              1,
              0x201
          },

          motor_2_{
              2,
              0x202
          },

          motor_3_{
              3,
              0x203
          },

          // ======================================================
          // Pitch 位置外环
          // ======================================================

          pitch_position_pid_(
              8.0,
              0.0,
              0.0),

          // ======================================================
          // Pitch 电机1速度内环
          // ======================================================

          pitch_speed_pid_1_(
              0.02,
              0.0,
              0.0),

          // ======================================================
          // Pitch 电机2速度内环
          // ======================================================

          pitch_speed_pid_2_(
              0.02,
              0.0,
              0.0),

          // ======================================================
          // Yaw 位置外环
          // ======================================================

          yaw_position_pid_(
              8.0,
              0.0,
              0.0),

          // ======================================================
          // Yaw 速度内环
          // ======================================================

          yaw_speed_pid_(
              0.02,
              0.0,
              0.0),

          // ======================================================
          // 三个速度低通滤波器
          // ======================================================

          speed_filter_1_(0.1),
          speed_filter_2_(0.1),
          speed_filter_3_(0.1)
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
            << "========================================"
            << '\n';

        std::cout
            << "M2006 three motor control system started."
            << '\n';

        std::cout
            << "========================================"
            << '\n';


        print_motor_info(
            "Pitch Motor 1",
            motor_1_);

        print_motor_info(
            "Pitch Motor 2",
            motor_2_);

        print_motor_info(
            "Yaw Motor",
            motor_3_);


        std::cout
            << "CAN TX ID = 0x200"
            << '\n';

        std::cout
            << "========================================"
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
        // 2. 获取 DR16 摇杆
        //
        // 右摇杆：
        //
        // Y -> Pitch
        // X -> Yaw
        //
        // 当前：
        //
        // Pitch：
        // physical right -> positive
        //
        // Yaw：
        // physical right -> positive
        // ========================================================

        double pitch_input = 0.0;

        double yaw_input = 0.0;


        if (dr16_valid)
        {
            // Pitch
            pitch_input =
                -dr16_.joystick_right().y();

            // Yaw
            yaw_input =
                dr16_.joystick_right().x();
        }


        // ========================================================
        // 3. 摇杆死区
        // ========================================================

        constexpr double kJoystickDeadzone =
            0.05;


        if (std::abs(pitch_input) <
            kJoystickDeadzone)
        {
            pitch_input = 0.0;
        }


        if (std::abs(yaw_input) <
            kJoystickDeadzone)
        {
            yaw_input = 0.0;
        }


        // ========================================================
        // 4. 摇杆转换为目标速度
        // ========================================================

        constexpr double kMaxPitchSpeed =
            10.0;

        constexpr double kMaxYawSpeed =
            10.0;


        const double pitch_target_velocity =
            pitch_input *
            kMaxPitchSpeed;


        const double yaw_target_velocity =
            yaw_input *
            kMaxYawSpeed;


        // ========================================================
        // 5. 积分得到目标角度
        //
        // Pitch：
        // 两个电机共享同一个 Pitch 目标角度
        //
        // Yaw：
        // 使用独立目标角度
        // ========================================================

        if (pitch_initialized_)
        {
            pitch_target_angle_ +=
                pitch_target_velocity *
                dt;
        }


        if (yaw_initialized_)
        {
            yaw_target_angle_ +=
                yaw_target_velocity *
                dt;
        }


        // ========================================================
        // 6. 读取三个 M2006 实际速度
        // ========================================================

        const double motor_1_speed =
            speed_filter_1_.update(
                motor_1_.speed);


        const double motor_2_speed =
            speed_filter_2_.update(
                motor_2_.speed);


        const double motor_3_speed =
            speed_filter_3_.update(
                motor_3_.speed);


        // ========================================================
        // 7. Pitch 位置外环
        //
        // 当前阶段：
        //
        // M2006 #1 -> Pitch
        // M2006 #2 -> Pitch
        //
        // 两个电机都追踪同一个 Pitch 目标角度。
        //
        // 注意：
        //
        // 这里暂时没有加入同步补偿。
        //
        // 下一阶段加入：
        //
        // sync_error =
        //     motor_1_.angle
        //     -
        //     motor_2_.angle
        // ========================================================

        double motor_1_target_speed = 0.0;

        double motor_2_target_speed = 0.0;


        if (pitch_initialized_)
        {
            motor_1_target_speed =
                pitch_position_pid_.update(
                    pitch_target_angle_,
                    motor_1_.angle,
                    dt);


            motor_2_target_speed =
                pitch_position_pid_.update(
                    pitch_target_angle_,
                    motor_2_.angle,
                    dt);
        }


        // ========================================================
        // 8. Yaw 位置外环
        // ========================================================

        double motor_3_target_speed = 0.0;


        if (yaw_initialized_)
        {
            motor_3_target_speed =
                yaw_position_pid_.update(
                    yaw_target_angle_,
                    motor_3_.angle,
                    dt);
        }


        // ========================================================
        // 9. Pitch 电机1速度内环
        // ========================================================

        double motor_1_current = 0.0;


        if (pitch_initialized_)
        {
            motor_1_current =
                pitch_speed_pid_1_.update(
                    motor_1_target_speed,
                    motor_1_speed,
                    dt);
        }


        // ========================================================
        // 10. Pitch 电机2速度内环
        // ========================================================

        double motor_2_current = 0.0;


        if (pitch_initialized_)
        {
            motor_2_current =
                pitch_speed_pid_2_.update(
                    motor_2_target_speed,
                    motor_2_speed,
                    dt);
        }


        // ========================================================
        // 11. Yaw 电机速度内环
        // ========================================================

        double motor_3_current = 0.0;


        if (yaw_initialized_)
        {
            motor_3_current =
                yaw_speed_pid_.update(
                    motor_3_target_speed,
                    motor_3_speed,
                    dt);
        }


        // ========================================================
        // 12. C610 电流限制
        //
        // C610 控制电流范围：
        //
        // -10000 ~ 10000
        //
        // 这里先人为限制到比较小的范围，
        // 防止第一次测试时输出过大。
        //
        // 后续调试 PID 时再逐步增加。
        // ========================================================

        constexpr double kMaxCurrent =
            2000.0;


        motor_1_current =
            clamp(
                motor_1_current,
                -kMaxCurrent,
                kMaxCurrent);


        motor_2_current =
            clamp(
                motor_2_current,
                -kMaxCurrent,
                kMaxCurrent);


        motor_3_current =
            clamp(
                motor_3_current,
                -kMaxCurrent,
                kMaxCurrent);


        // ========================================================
        // 13. 保存三个电机控制电流
        // ========================================================

        motor_1_.command_current =
            static_cast<int>(
                motor_1_current);


        motor_2_.command_current =
            static_cast<int>(
                motor_2_current);


        motor_3_.command_current =
            static_cast<int>(
                motor_3_current);


        // ========================================================
        // 14. CAN 发送
        //
        // 控制频率：
        //
        // PID：
        // 1 kHz
        //
        // CAN：
        // 100 Hz
        // ========================================================

        static int send_count = 0;


        ++send_count;


        if (send_count >= 10)
        {
            send_count = 0;

            send_motor_commands();
        }


        // ========================================================
        // 15. 调试输出
        // ========================================================

        static int debug_count = 0;


        ++debug_count;


        if (debug_count >= 100)
        {
            debug_count = 0;


            std::cout
                << "DR16="
                << dr16_valid

                << " | Pitch input="
                << pitch_input

                << " | Yaw input="
                << yaw_input

                << " | Pitch target="
                << pitch_target_angle_

                << " | M1 angle="
                << motor_1_.angle

                << " | M2 angle="
                << motor_2_.angle

                << " | Yaw target="
                << yaw_target_angle_

                << " | M3 angle="
                << motor_3_.angle

                << " | I1="
                << motor_1_.command_current

                << " | I2="
                << motor_2_.command_current

                << " | I3="
                << motor_3_.command_current

                << '\n';
        }
    }


private:

    // ============================================================
    // 数值限制
    // ============================================================

    static double clamp(
        double value,
        double min_value,
        double max_value)
    {
        if (value < min_value)
        {
            return min_value;
        }

        if (value > max_value)
        {
            return max_value;
        }

        return value;
    }


    // ============================================================
    // 打印电机信息
    // ============================================================

    void print_motor_info(
        const char* name,
        const M2006Motor& motor)
    {
        std::cout
            << name
            << '\n';

        std::cout
            << "  Motor ID = "
            << motor.id
            << '\n';

        std::cout
            << "  RX CAN ID = 0x"
            << std::hex
            << motor.recv_id
            << std::dec
            << '\n';
    }


    // ============================================================
    // UART 接收
    //
    // DR16 使用 RMCS 已经封装好的 DBUS。
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
    //
    // 一个 callback 管三个 M2006。
    // ============================================================

    void can_receive_callback(
        const Spec::Can& can,
        const View::Can& data) override
    {
        // --------------------------------------------------------
        // 只使用 CAN1
        // --------------------------------------------------------

        if (can != Spec::kCans.kCan1)
        {
            return;
        }


        // --------------------------------------------------------
        // M2006 反馈必须是 8 字节
        // --------------------------------------------------------

        if (data.can_data.size() != 8)
        {
            return;
        }


        // --------------------------------------------------------
        // M2006 ID1
        // --------------------------------------------------------

        if (data.can_id ==
            motor_1_.recv_id)
        {
            parse_motor_feedback(
                motor_1_,
                data);

            return;
        }


        // --------------------------------------------------------
        // M2006 ID2
        // --------------------------------------------------------

        if (data.can_id ==
            motor_2_.recv_id)
        {
            parse_motor_feedback(
                motor_2_,
                data);

            return;
        }


        // --------------------------------------------------------
        // M2006 ID3
        // --------------------------------------------------------

        if (data.can_id ==
            motor_3_.recv_id)
        {
            parse_motor_feedback(
                motor_3_,
                data);

            return;
        }
    }


    // ============================================================
    // 解析 M2006/C610 反馈
    //
    // DATA[0] DATA[1]
    //     转子机械角度
    //
    // DATA[2] DATA[3]
    //     转子转速 RPM
    //
    // DATA[4] DATA[5]
    //     实际转矩电流
    //
    // DATA[6]
    //     温度
    // ============================================================

    void parse_motor_feedback(
        M2006Motor& motor,
        const View::Can& data)
    {
        // --------------------------------------------------------
        // 保存上一次编码器
        // --------------------------------------------------------

        motor.last_encoder =
            motor.encoder;


        // --------------------------------------------------------
        // 读取编码器
        // --------------------------------------------------------

        motor.encoder =
            static_cast<std::uint16_t>(
                (static_cast<std::uint16_t>(
                    static_cast<std::uint8_t>(
                        data.can_data[0]))
                 << 8)
                |
                static_cast<std::uint16_t>(
                    static_cast<std::uint8_t>(
                        data.can_data[1])));


        // --------------------------------------------------------
        // 读取速度
        // --------------------------------------------------------

        const std::int16_t speed_rpm =
            static_cast<std::int16_t>(
                (static_cast<std::uint16_t>(
                    static_cast<std::uint8_t>(
                        data.can_data[2]))
                 << 8)
                |
                static_cast<std::uint16_t>(
                    static_cast<std::uint8_t>(
                        data.can_data[3])));


        motor.speed =
            static_cast<double>(
                speed_rpm);


        // --------------------------------------------------------
        // 读取实际转矩电流
        // --------------------------------------------------------

        const std::int16_t current =
            static_cast<std::int16_t>(
                (static_cast<std::uint16_t>(
                    static_cast<std::uint8_t>(
                        data.can_data[4]))
                 << 8)
                |
                static_cast<std::uint16_t>(
                    static_cast<std::uint8_t>(
                        data.can_data[5])));


        motor.current =
            static_cast<double>(
                current);


        // --------------------------------------------------------
        // 读取温度
        // --------------------------------------------------------

        motor.temperature =
            static_cast<std::uint8_t>(
                data.can_data[6]);


        // --------------------------------------------------------
        // 第一次收到反馈
        //
        // 直接把当前位置作为初始零点。
        // --------------------------------------------------------

        if (!motor.initialized)
        {
            motor.initialized =
                true;


            motor.last_encoder =
                motor.encoder;


            motor.angle = 0.0;

            motor.last_angle = 0.0;


            std::cout
                << "M2006 ID "
                << motor.id
                << " feedback detected."
                << " CAN ID = 0x"
                << std::hex
                << data.can_id
                << std::dec
                << '\n';


            update_initialization_state();


            return;
        }


        // --------------------------------------------------------
        // 计算编码器变化量
        //
        // M2006 编码器：
        //
        // 0 ~ 8191
        //
        // 跨越 0 / 8191 时需要处理。
        // --------------------------------------------------------

        int encoder_delta =
            static_cast<int>(
                motor.encoder)
            -
            static_cast<int>(
                motor.last_encoder);


        // --------------------------------------------------------
        // 正向跨圈
        // --------------------------------------------------------

        if (encoder_delta <
            -4096)
        {
            encoder_delta +=
                8192;
        }


        // --------------------------------------------------------
        // 反向跨圈
        // --------------------------------------------------------

        else if (encoder_delta >
                 4096)
        {
            encoder_delta -=
                8192;
        }


        // --------------------------------------------------------
        // 编码器：
        //
        // 8192 -> 360°
        // --------------------------------------------------------

        const double delta_angle =
            static_cast<double>(
                encoder_delta)
            *
            360.0
            /
            8192.0;


        // --------------------------------------------------------
        // 连续角度
        // --------------------------------------------------------

        motor.last_angle =
            motor.angle;


        motor.angle +=
            delta_angle;
    }


    // ============================================================
    // 检查三个电机是否都初始化完成
    // ============================================================

    void update_initialization_state()
    {
        // --------------------------------------------------------
        // Pitch：
        //
        // 两个电机都收到反馈以后，
        // 才允许 Pitch 控制。
        // --------------------------------------------------------

        if (motor_1_.initialized &&
            motor_2_.initialized)
        {
            if (!pitch_initialized_)
            {
                pitch_target_angle_ =
                    0.0;

                pitch_initialized_ =
                    true;


                std::cout
                    << "Pitch motors initialized."
                    << '\n';
            }
        }


        // --------------------------------------------------------
        // Yaw：
        //
        // M2006 ID3 收到反馈以后，
        // 才允许 Yaw 控制。
        // --------------------------------------------------------

        if (motor_3_.initialized)
        {
            if (!yaw_initialized_)
            {
                yaw_target_angle_ =
                    0.0;

                yaw_initialized_ =
                    true;


                std::cout
                    << "Yaw motor initialized."
                    << '\n';
            }
        }
    }


    // ============================================================
    // 发送三个 M2006 控制命令
    //
    // C610：
    //
    // CAN ID = 0x200
    //
    // DATA[0] DATA[1]
    //     Motor ID1
    //
    // DATA[2] DATA[3]
    //     Motor ID2
    //
    // DATA[4] DATA[5]
    //     Motor ID3
    //
    // DATA[6] DATA[7]
    //     Motor ID4
    // ============================================================

    void send_motor_commands()
    {
        std::array<std::byte, 8> can_data{};


        // --------------------------------------------------------
        // Motor 1
        // --------------------------------------------------------

        write_int16_to_can(
            can_data,
            0,
            motor_1_.command_current);


        // --------------------------------------------------------
        // Motor 2
        // --------------------------------------------------------

        write_int16_to_can(
            can_data,
            2,
            motor_2_.command_current);


        // --------------------------------------------------------
        // Motor 3
        // --------------------------------------------------------

        write_int16_to_can(
            can_data,
            4,
            motor_3_.command_current);


        // --------------------------------------------------------
        // Motor 4 没有使用
        //
        // DATA[6]
        // DATA[7]
        //
        // 保持 0。
        // --------------------------------------------------------

        can_data[6] =
            std::byte{0};

        can_data[7] =
            std::byte{0};


        // --------------------------------------------------------
        // 构造 CAN 数据
        // --------------------------------------------------------

        librmcs::data::CanDataView data{
            0x200,

            std::span<const std::byte>(
                can_data.data(),
                can_data.size())
        };


        // --------------------------------------------------------
        // 发送 CAN
        // --------------------------------------------------------

        board_->start_transmit()
            .can_transmit(
                Spec::kCans.kCan1,
                data);
    }


    // ============================================================
    // 把 int16_t 写入 CAN 高低字节
    // ============================================================

    void write_int16_to_can(
        std::array<std::byte, 8>& data,
        std::size_t index,
        int value)
    {
        // --------------------------------------------------------
        // 防止超出 C610 电流范围
        // --------------------------------------------------------

        if (value > 10000)
        {
            value = 10000;
        }

        if (value < -10000)
        {
            value = -10000;
        }


        const std::int16_t current =
            static_cast<std::int16_t>(
                value);


        const std::uint16_t raw =
            static_cast<std::uint16_t>(
                current);


        data[index] =
            static_cast<std::byte>(
                (raw >> 8) & 0xFF);


        data[index + 1] =
            static_cast<std::byte>(
                raw & 0xFF);
    }


private:

    // ============================================================
    // DR16
    // ============================================================

    rmcs_core::hardware::device::Dr16 dr16_;


    // ============================================================
    // 三个 M2006
    //
    // 1、2 -> Pitch
    // 3    -> Yaw
    // ============================================================

    M2006Motor motor_1_;

    M2006Motor motor_2_;

    M2006Motor motor_3_;


    // ============================================================
    // Pitch 位置外环
    // ============================================================

    PidController pitch_position_pid_;


    // ============================================================
    // Pitch 电机1速度内环
    // ============================================================

    PidController pitch_speed_pid_1_;


    // ============================================================
    // Pitch 电机2速度内环
    // ============================================================

    PidController pitch_speed_pid_2_;


    // ============================================================
    // Yaw 位置外环
    // ============================================================

    PidController yaw_position_pid_;


    // ============================================================
    // Yaw 速度内环
    // ============================================================

    PidController yaw_speed_pid_;


    // ============================================================
    // 三个速度低通滤波器
    // ============================================================

    LowPassFilter speed_filter_1_;

    LowPassFilter speed_filter_2_;

    LowPassFilter speed_filter_3_;


    // ============================================================
    // Pitch 目标角度
    // ============================================================

    double pitch_target_angle_ =
        0.0;


    // ============================================================
    // Yaw 目标角度
    // ============================================================

    double yaw_target_angle_ =
        0.0;


    // ============================================================
    // Pitch 是否已经初始化
    // ============================================================

    bool pitch_initialized_ =
        false;


    // ============================================================
    // Yaw 是否已经初始化
    // ============================================================

    bool yaw_initialized_ =
        false;


    // ============================================================
    // C Board
    // ============================================================

    std::unique_ptr<
        librmcs::board::CBoard>
        board_;
};


// ================================================================
// main
// ================================================================

int main()
{
    MotorControlSystem system;


    // ------------------------------------------------------------
    // 启动 C Board
    // ------------------------------------------------------------

    system.start();


    // ------------------------------------------------------------
    // 1 kHz 控制循环
    // ------------------------------------------------------------

    while (true)
    {
        system.update();


        std::this_thread::sleep_for(
            1ms);
    }


    return 0;
}
