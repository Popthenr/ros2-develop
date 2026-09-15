#include <algorithm>
#include <deque>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class SignalFilter : public rclcpp::Node
{
public:
    SignalFilter()
        : Node("signal_filter"),
          lowpass_value_(0.0),
          initialized_(false)
    {
        // 订阅带噪声的原始信号
        subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/noisy_signal",
            10,
            std::bind(&SignalFilter::filter_signal, this, std::placeholders::_1));

        // 发布低通滤波结果
        lowpass_publisher_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/lowpass_signal", 10);

        // 发布中值滤波结果
        median_publisher_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/median_signal", 10);
    }

private:
    void filter_signal(const std_msgs::msg::Float64::SharedPtr msg)
    {
        double input = msg->data;

        // =========================
        // 1. 一阶低通滤波
        // =========================
        //
        // y[n] = alpha * x[n]
        //      + (1 - alpha) * y[n-1]
        //
        // alpha 越小，滤波越强，但响应越慢。
        const double alpha = 0.1;

        if (!initialized_)
        {
            lowpass_value_ = input;
            initialized_ = true;
        }
        else
        {
            lowpass_value_ =
                alpha * input +
                (1.0 - alpha) * lowpass_value_;
        }

        // =========================
        // 2. 中值滤波
        // =========================
        //
        // 使用最近 5 个数据点
        median_window_.push_back(input);

        if (median_window_.size() > 5)
        {
            median_window_.pop_front();
        }

        std::vector<double> sorted_window(
            median_window_.begin(),
            median_window_.end());

        std::sort(
            sorted_window.begin(),
            sorted_window.end());

        double median_value =
            sorted_window[sorted_window.size() / 2];

        // =========================
        // 3. 发布低通滤波结果
        // =========================
        std_msgs::msg::Float64 lowpass_msg;
        lowpass_msg.data = lowpass_value_;

        lowpass_publisher_->publish(lowpass_msg);

        // =========================
        // 4. 发布中值滤波结果
        // =========================
        std_msgs::msg::Float64 median_msg;
        median_msg.data = median_value;

        median_publisher_->publish(median_msg);
    }

    // 订阅者
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;

    // 发布者
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        lowpass_publisher_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        median_publisher_;

    // 低通滤波上一次输出
    double lowpass_value_;

    // 是否已经处理过第一个数据
    bool initialized_;

    // 中值滤波窗口
    std::deque<double> median_window_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SignalFilter>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}