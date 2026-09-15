#include <chrono>
#include <cmath>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class SignalPublisher : public rclcpp::Node
{
public:
    SignalPublisher()
        : Node("signal_publisher"),
          generator_(std::random_device{}()),
          noise_distribution_(0.0, 0.01)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/noisy_signal", 10);

        timer_ = this->create_wall_timer(
            1ms,
            std::bind(&SignalPublisher::publish_signal, this));

        start_time_ = this->now();
    }

private:
    void publish_signal()
    {
        double t = (this->now() - start_time_).seconds();

        // 正弦信号参数
        const double amplitude = 1.0;
        const double frequency = 20.0;

        // 原始正弦信号
        double signal = amplitude *
                        std::sin(2.0 * M_PI * frequency * t);

        // 高斯白噪声，标准差为信号幅值的 1%
        double noise = noise_distribution_(generator_);

        // 加噪后的信号
        double noisy_signal = signal + noise;

        std_msgs::msg::Float64 msg;
        msg.data = noisy_signal;

        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time start_time_;

    std::mt19937 generator_;
    std::normal_distribution<double> noise_distribution_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SignalPublisher>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}

