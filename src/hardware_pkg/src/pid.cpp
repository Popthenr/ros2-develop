#include "pid.hpp"

Pid::Pid(double kp, double ki, double kd)
    : kp(kp),
      ki(ki),
      kd(kd)
{
}

double Pid::update(double error, double dt)
{
    if (dt <= 0.0)
    {
        return 0.0;
    }

    // 积分项
    integral_ += error * dt;

    // 防止积分饱和
    integral_ = std::clamp(
        integral_,
        integral_min,
        integral_max);

    // 微分项
    const double derivative =
        (error - previous_error_) / dt;

    // PID 输出
    double output =
        kp * error
        + ki * integral_
        + kd * derivative;

    // 输出限幅
    output = std::clamp(
        output,
        output_min,
        output_max);

    // 保存本次误差
    previous_error_ = error;

    return output;
}

void Pid::reset()
{
    integral_ = 0.0;
    previous_error_ = 0.0;
}