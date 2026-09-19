#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class TargetAngle : public rclcpp::Node
{
public:
    TargetAngle()
        : Node("target_angle"),
          elapsed_time_(0.0)
    {
        angle_pub_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/target_angle",
                10);

        timer_ = this->create_wall_timer(
            10ms,
            std::bind(
                &TargetAngle::publishAngle,
                this));

        RCLCPP_INFO(
            this->get_logger(),
            "Target angle source started.");
    }

private:
    void publishAngle()
    {
        constexpr double dt = 0.01;

        elapsed_time_ += dt;

        if (elapsed_time_ >= 40.0)
        {
            elapsed_time_ = 0.0;
        }

        double target_angle = 0.0;

        if (elapsed_time_ < 10.0)
        {
            target_angle = 0.0;
        }
        else if (elapsed_time_ < 20.0)
        {
            target_angle = M_PI / 2;
        }
        else if (elapsed_time_ < 30.0)
        {
            target_angle = M_PI;
        }
        else
        {
            target_angle = 3 * M_PI / 2;
        }

        std_msgs::msg::Float64 msg;
        msg.data = target_angle;

        angle_pub_->publish(msg);
    }

    double elapsed_time_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        angle_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<TargetAngle>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}