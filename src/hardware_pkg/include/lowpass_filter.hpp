#pragma once

class LowPassFilter
{
public:
    explicit LowPassFilter(double alpha);

    // 输入一个新的数据，返回滤波后的数据
    double update(double input);

    // 重置滤波器
    void reset(double value = 0.0);

private:
    double alpha_;//系数
    double output_{0.0};//旧输出
    bool initialized_{false};
};