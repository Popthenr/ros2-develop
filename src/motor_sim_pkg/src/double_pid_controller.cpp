#include <chrono>
#include <memory>
#include <algorithm>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class DoublePIDController : public rclcpp::Node
{
public:
    DoublePIDController()
        : Node("double_pid_controller"),
          target_angle_(0.0),
          actual_angle_(0.0),
          target_velocity_(0.0),
          actual_velocity_(0.0),
          angle_integral_(0.0),
          angle_previous_error_(0.0),
          velocity_integral_(0.0),
          velocity_previous_error_(0.0)
    {
        // 角度环参数
        angle_Kp_ = 1.0;
        angle_Ki_ = 0.0;
        angle_Kd_ = 0.0;

        // 速度环参数
        velocity_Kp_ = 0.5;
        velocity_Ki_ = 1.0;
        velocity_Kd_ = 0.001;

        max_target_velocity_ = 20.0;
        max_torque_ = 20.0;

        // 订阅目标角度
        target_angle_sub_ =
            this->create_subscription<std_msgs::msg::Float64>(
                "/target_angle",
                10,
                std::bind(
                    &DoublePIDController::targetAngleCallback,
                    this,
                    std::placeholders::_1));

        // 订阅实际角度
        angle_sub_ =
            this->create_subscription<std_msgs::msg::Float64>(
                "/motor_angle",
                10,
                std::bind(
                    &DoublePIDController::angleCallback,
                    this,
                    std::placeholders::_1));

        // 订阅实际速度
        speed_sub_ =
            this->create_subscription<std_msgs::msg::Float64>(
                "/motor_speed",
                10,
                std::bind(
                    &DoublePIDController::speedCallback,
                    this,
                    std::placeholders::_1));

        // 发布角度PID产生的目标速度
        target_velocity_pub_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/target_velocity_double",
                10);

        // 发布速度PID产生的力矩
        torque_pub_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/motor_torque",
                10);

        // 1000 Hz控制循环
        timer_ =
            this->create_wall_timer(
                1ms,
                std::bind(
                    &DoublePIDController::update,
                    this));

        RCLCPP_INFO(
            this->get_logger(),
            "Double PID controller started.");
        
    }

private:
    double normalizeAngle(double angle)
    {
    const double two_pi = 2.0 * M_PI;

    angle = std::fmod(angle, two_pi);

    if (angle < 0.0)
    {
        angle += two_pi;
    }

    return angle;
    }

    double calculateMajorArcError(double target,double actual)
    {
    const double two_pi = 2.0 * M_PI;

    // 先把目标角度和实际角度统一到 0~2π
    target = normalizeAngle(target);
    actual = normalizeAngle(actual);

    // 计算普通角度差
    double error = target - actual;

    // 把误差转换到 [-π, π)
    error = std::fmod(
        error + M_PI,
        two_pi);

    if (error < 0.0)
    {
        error += two_pi;
    }

    error -= M_PI;

    // --------------------------------
    // 现在 error 是最短角度误差
    // --------------------------------

    // 如果最短误差为正，
    // 优弧就应该反方向走
    if (error > 0.0)
    {
        error -= two_pi;
    }
    // 如果最短误差为负，
    // 优弧就应该反方向走
    else if (error < 0.0)
    {
        error += two_pi;
    }

    return error;
    }

    void targetAngleCallback(
        const std_msgs::msg::Float64::SharedPtr msg)
    {
        target_angle_ = msg->data;
    }

    void angleCallback(
        const std_msgs::msg::Float64::SharedPtr msg)
    {
        actual_angle_ = msg->data;
    }

    void speedCallback(
        const std_msgs::msg::Float64::SharedPtr msg)
    {
        actual_velocity_ = msg->data;
    }

    void update()
    {
        constexpr double dt = 0.001;

        // ==========================
        // 第一层：角度 PID
        // ==========================

        double angle_error =
    calculateMajorArcError(
        target_angle_,
        actual_angle_);

        angle_integral_ +=
            angle_error * dt;

        angle_integral_ =
            std::clamp(
                angle_integral_,
                -20.0,
                20.0);

        double angle_derivative =
            (angle_error -
             angle_previous_error_) / dt;

        double target_velocity =
            angle_Kp_ * angle_error
            + angle_Ki_ * angle_integral_
            + angle_Kd_ * angle_derivative;

        target_velocity =
            std::clamp(
                target_velocity,
                -max_target_velocity_,
                max_target_velocity_);

        angle_previous_error_ =
            angle_error;

        // ==========================
        // 第二层：速度 PID
        // ==========================

        double velocity_error =
            target_velocity -
            actual_velocity_;

        velocity_integral_ +=
            velocity_error * dt;

        velocity_integral_ =
            std::clamp(
                velocity_integral_,
                -20.0,
                20.0);

        double velocity_derivative =
            (velocity_error -
             velocity_previous_error_) / dt;

        double torque =
            velocity_Kp_ * velocity_error
            + velocity_Ki_ * velocity_integral_
            + velocity_Kd_ * velocity_derivative;

        torque =
            std::clamp(
                torque,
                -max_torque_,
                max_torque_);

        velocity_previous_error_ =
            velocity_error;

        // 发布角度环输出的目标速度
        std_msgs::msg::Float64 velocity_msg;
        velocity_msg.data = target_velocity;

        target_velocity_pub_->publish(
            velocity_msg);

        // 发布速度环输出的力矩
        std_msgs::msg::Float64 torque_msg;
        torque_msg.data = torque;

        torque_pub_->publish(
            torque_msg);
    }

    // 当前状态
    double target_angle_;
    double actual_angle_;

    double target_velocity_;
    double actual_velocity_;

    // 角度PID
    double angle_Kp_;
    double angle_Ki_;
    double angle_Kd_;

    double angle_integral_;
    double angle_previous_error_;

    // 速度PID
    double velocity_Kp_;
    double velocity_Ki_;
    double velocity_Kd_;

    double velocity_integral_;
    double velocity_previous_error_;

    double max_target_velocity_;
    double max_torque_;

    // ROS2
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
        target_angle_sub_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
        angle_sub_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
        speed_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        target_velocity_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        torque_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<DoublePIDController>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}