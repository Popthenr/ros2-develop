#include "lowpass_filter.hpp"

LowPassFilter::LowPassFilter(double alpha)
    : alpha_(alpha)
{
    if (alpha_ < 0.0)
    {
        alpha_ = 0.0;
    }

    if (alpha_ > 1.0)
    {
        alpha_ = 1.0;
    }
}

double LowPassFilter::update(double input)
{
    // 第一次输入直接作为初始值
    if (!initialized_)
    {
        output_ = input;
        initialized_ = true;

        return output_;
    }

    // 一阶低通滤波
    output_ += alpha_ * (input - output_);

    return output_;
}

void LowPassFilter::reset(double value)
{
    output_ = value;
    initialized_ = true;
}