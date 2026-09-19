#include <chrono>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class PIDController : public rclcpp::Node
{
public:
    PIDController()
        : Node("pid_controller"),
          target_velocity_(0.0),
          actual_velocity_(0.0),
          integral_(0.0),
          previous_error_(0.0)
    {
        // PID参数
        Kp_ = 0.5;
        Ki_ = 1.0;
        Kd_ = 0.001;

        // 最大输出力矩
        max_torque_ = 20.0;

        // 订阅目标速度
        target_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/target_velocity",
            10,
            std::bind(
                &PIDController::targetCallback,
                this,
                std::placeholders::_1));

        // 订阅电机实际速度
        speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/motor_speed",
            10,
            std::bind(
                &PIDController::speedCallback,
                this,
                std::placeholders::_1));

        // 发布控制力矩
        torque_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/motor_torque",
            10);

        // 1 ms执行一次，即1000 Hz
        timer_ = this->create_wall_timer(
            1ms,
            std::bind(
                &PIDController::update,
                this));

        RCLCPP_INFO(
            this->get_logger(),
            "PID controller started.");
    }

private:

    // 接收目标速度
    void targetCallback(
        const std_msgs::msg::Float64::SharedPtr msg)
    {
        target_velocity_ = msg->data;
    }

    // 接收电机实际速度
    void speedCallback(
        const std_msgs::msg::Float64::SharedPtr msg)
    {
        actual_velocity_ = msg->data;
    }

    // PID计算
    void update()
    {
        constexpr double dt = 0.001;  // 1 ms

        // 计算速度误差
        double error =
            target_velocity_ - actual_velocity_;

        // 积分项
        integral_ += error * dt;

        // 防止积分饱和
        integral_ = std::clamp(
            integral_,
            -20.0,
            20.0);

        // 微分项
        double derivative =
            (error - previous_error_) / dt;

        // PID输出
        double torque =
            Kp_ * error
            + Ki_ * integral_
            + Kd_ * derivative;

        // 限制最大控制力矩
        torque = std::clamp(
            torque,
            -max_torque_,
            max_torque_);

        // 保存本次误差
        previous_error_ = error;

        // 发布控制力矩
        std_msgs::msg::Float64 msg;
        msg.data = torque;

        torque_pub_->publish(msg);
    }

    // 目标速度
    double target_velocity_;

    // 电机实际速度
    double actual_velocity_;

    // PID参数
    double Kp_;
    double Ki_;
    double Kd_;

    // PID内部状态
    double integral_;
    double previous_error_;

    // 最大输出力矩
    double max_torque_;

    // ROS2
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
        target_sub_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
        speed_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        torque_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<PIDController>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}