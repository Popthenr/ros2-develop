#include <chrono>
#include <cmath>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class MotorSimulator : public rclcpp::Node
{
public:
    MotorSimulator()
        : Node("motor_simulator"),
          theta_(0.0),
          omega_(0.0),
          torque_(0.0)
    {
        // ==============================
        // 电机参数
        // ==============================
        // 转动惯量 J，单位 kg·m²
        J_ = 0.01;

        // 粘性阻尼系数 B，单位 N·m·s/rad
        B_ = 0.1;

        // ==============================
        // 接收控制力矩
        // ==============================
        torque_subscription_ =
            this->create_subscription<std_msgs::msg::Float64>(
                "motor_torque",
                10,
                std::bind(
                    &MotorSimulator::torque_callback,
                    this,
                    std::placeholders::_1));

        // ==============================
        // 发布电机角速度
        // ==============================
        speed_publisher_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "motor_speed",
                10);

        // ==============================
        // 发布电机角度
        // ==============================
        angle_publisher_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "motor_angle",
                10);

        // ==============================
        // 状态更新频率：1000 Hz
        // dt = 1 ms
        // ==============================
        timer_ = this->create_wall_timer(
            1ms,
            std::bind(
                &MotorSimulator::update_state,
                this));

        RCLCPP_INFO(
            this->get_logger(),
            "Motor simulator started.");
        
        RCLCPP_INFO(
            this->get_logger(),
            "State update frequency: 1000 Hz");
    }

private:

    // ==========================================
    // 接收控制力矩
    // ==========================================
    void torque_callback(
        const std_msgs::msg::Float64::SharedPtr msg)
    {
        torque_ = msg->data;
    }

    // ==========================================
    // 电机动力学计算
    //
    // J * omega_dot = T - B * omega
    //
    // omega_dot = (T - B * omega) / J
    //
    // theta_dot = omega
    // ==========================================
    void update_state()
    {
        constexpr double dt = 0.001;  // 1 ms

        // 计算角加速度
        double omega_dot =
            (torque_ - B_ * omega_) / J_;

        // 积分得到角速度
        omega_ += omega_dot * dt;
        if (omega_ >100.0)
        {
            omega_ =100.0;
        }
        else if (omega_ < -100.0)
        {
            omega_ = -100.0;
        }

        // 积分得到角度
        theta_ += omega_ * dt;

        // ==============================
        // 发布角速度
        // ==============================
        std_msgs::msg::Float64 speed_msg;
        speed_msg.data = omega_;

        speed_publisher_->publish(speed_msg);

        // ==============================
        // 发布角度
        // ==============================
        std_msgs::msg::Float64 angle_msg;
        angle_msg.data = theta_;

        angle_publisher_->publish(angle_msg);
    }

    // ==============================
    // 电机参数
    // ==============================
    double J_;
    double B_;

    // ==============================
    // 电机状态
    // ==============================
    double theta_;   // 角度 rad
    double omega_;   // 角速度 rad/s

    // 当前控制力矩
    double torque_;  // N·m

    // ==============================
    // ROS 2 通信
    // ==============================
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
        torque_subscription_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        speed_publisher_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        angle_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<MotorSimulator>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
