#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class TargetVelocity : public rclcpp::Node
{
public:
    TargetVelocity()
        : Node("target_velocity"),
          elapsed_time_(0.0)
    {
        // 发布目标速度
        velocity_pub_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/target_velocity",
                10);

        // 100 Hz 发布
        timer_ = this->create_wall_timer(
            10ms,
            std::bind(
                &TargetVelocity::publishVelocity,
                this));

        RCLCPP_INFO(
            this->get_logger(),
            "Target velocity source started.");
    }

private:
    void publishVelocity()
    {
        constexpr double dt = 0.01;  // 10 ms

        elapsed_time_ += dt;

        // 40 秒循环
        if (elapsed_time_ >= 40.0)
        {
            elapsed_time_ = 0.0;
        }

        double target_velocity = 0.0;

        // 0 ~ 10 s：0 rad/s
        if (elapsed_time_ < 10.0)
        {
            target_velocity = 0.0;
        }
        // 10 ~ 20 s：10 rad/s
        else if (elapsed_time_ < 20.0)
        {
            target_velocity = 10.0;
        }
        // 20 ~ 30 s：5 rad/s
        else if (elapsed_time_ < 30.0)
        {
            target_velocity = 5.0;
        }
        // 30 ~ 40 s：15 rad/s
        else
        {
            target_velocity = 15.0;
        }

        std_msgs::msg::Float64 msg;
        msg.data = target_velocity;

        velocity_pub_->publish(msg);
    }

    double elapsed_time_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        velocity_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<TargetVelocity>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}