#include <chrono>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class TorqueSource : public rclcpp::Node
{
public:
    TorqueSource()
        : Node("torque_source"),
          elapsed_time_(0.0)
    {
        // ==============================
        // 发布控制力矩
        // ==============================
        torque_publisher_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "motor_torque",
                10);

        // ==============================
        // 控制频率：100 Hz
        // dt = 10 ms
        // ==============================
        timer_ = this->create_wall_timer(
            10ms,
            std::bind(
                &TorqueSource::publish_torque,
                this));

        RCLCPP_INFO(
            this->get_logger(),
            "Torque source started.");

        RCLCPP_INFO(
            this->get_logger(),
            "Control frequency: 100 Hz");
    }

private:

    void publish_torque()
    {
        constexpr double dt = 0.01;  // 10 ms

        elapsed_time_ += dt;

        double torque = 0.0;

        // =====================================
        // 测试信号
        //
        // 0 ~ 2 s   :  0 N·m
        // 2 ~ 5 s   : +1 N·m
        // 5 ~ 7 s   :  0 N·m
        // 7 ~ 10 s  : -1 N·m
        // 10 s以后  : 0 N·m
        // =====================================

        if (elapsed_time_ >= 2.0 &&
            elapsed_time_ < 5.0)
        {
            torque = 1.0;
        }
        else if (elapsed_time_ >= 7.0 &&
                 elapsed_time_ < 10.0)
        {
            torque = -1.0;
        }
        else
        {
            torque = 0.0;
        }

        // 发布控制力矩
        std_msgs::msg::Float64 torque_msg;
        torque_msg.data = torque;

        torque_publisher_->publish(torque_msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        torque_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    double elapsed_time_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<TorqueSource>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}

