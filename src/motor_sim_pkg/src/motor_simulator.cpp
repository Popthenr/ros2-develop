#include <chrono>
#include <memory>

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
        // 电机参数
        J_ = 0.01;   // 转动惯量 kg·m²
        B_ = 0.1;    // 粘性阻尼系数 N·m·s/rad

        // 订阅控制力矩
        torque_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/motor_torque",
            10,
            std::bind(
                &MotorSimulator::torqueCallback,
                this,
                std::placeholders::_1));

        // 发布电机角速度
        speed_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/motor_speed",
            10);

        // 发布电机角度
        angle_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/motor_angle",
            10);

        // 1 ms 更新一次，即 1000 Hz
        timer_ = this->create_wall_timer(
            1ms,
            std::bind(
                &MotorSimulator::update,
                this));

        RCLCPP_INFO(
            this->get_logger(),
            "Motor simulator started.");
    }

private:
    // 接收控制力矩
    void torqueCallback(
        const std_msgs::msg::Float64::SharedPtr msg)
    {
        torque_ = msg->data;
    }

    // 电机动力学更新
    void update()
    {
        // 仿真步长
        constexpr double dt = 0.001;  // 1 ms

        /*
         * 电机动力学：
         *
         * J * omega_dot + B * omega = torque
         *
         * 所以：
         *
         * omega_dot = (torque - B * omega) / J
         */

        double omega_dot =
            (torque_ - B_ * omega_) / J_;

        // 数值积分得到角速度
        omega_ += omega_dot * dt;

        /*
         * 角度动力学：
         *
         * theta_dot = omega
         */
        theta_ += omega_ * dt;

        // 发布角速度
        std_msgs::msg::Float64 speed_msg;
        speed_msg.data = omega_;
        speed_pub_->publish(speed_msg);

        // 发布角度
        std_msgs::msg::Float64 angle_msg;
        angle_msg.data = theta_;
        angle_pub_->publish(angle_msg);
    }

    // 电机状态
    double theta_;   // 当前角度 rad
    double omega_;   // 当前角速度 rad/s
    double torque_;  // 当前输入力矩 N·m

    // 电机参数
    double J_;       // 转动惯量
    double B_;       // 粘性阻尼系数

    // ROS2
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr torque_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MotorSimulator>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}