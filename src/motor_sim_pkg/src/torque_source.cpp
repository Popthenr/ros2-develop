#include <chrono>
#include <memory>

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
        // 发布控制力矩
        torque_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/motor_torque",
            10);

        // 1000 Hz 发布测试输入
        timer_ = this->create_wall_timer(
            1ms,
            std::bind(
                &TorqueSource::publishTorque,
                this));

        RCLCPP_INFO(
            this->get_logger(),
            "Torque source started.");
    }

private:
    void publishTorque()
    {
        constexpr double dt = 0.001;  // 1 ms

        elapsed_time_ += dt;

        // 30 秒一个周期
        double t = elapsed_time_;

        if (t >= 30.0)
        {
            elapsed_time_ = 0.0;
            t = 0.0;
        }

        double torque = 0.0;

        /*
         * 30 秒循环阶跃输入：
         *
         * 0  ~ 5 s   :  0 Nm
         * 5  ~ 15 s  : +1 Nm
         * 15 ~ 20 s  :  0 Nm
         * 20 ~ 30 s  : -1 Nm
         *
         * 30 s 后重新开始
         */

        if (t >= 5.0 && t < 15.0)
        {
            torque = 1.0;
        }
        else if (t >= 20.0 && t < 30.0)
        {
            torque = -1.0;
        }

        std_msgs::msg::Float64 msg;
        msg.data = torque;

        torque_pub_->publish(msg);
    }

    double elapsed_time_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr torque_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TorqueSource>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}

