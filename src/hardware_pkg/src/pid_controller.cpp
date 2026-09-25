#include "pid_controller.hpp"

PidController::PidController(
    double kp,
    double ki,
    double kd)
    : pid_(kp, ki, kd)
{
}

double PidController::update(
    double target,
    double actual,
    double dt)
{
    const double error = target - actual;

    return pid_.update(error, dt);
}

void PidController::reset()
{
    pid_.reset();
}

Pid& PidController::pid()
{
    return pid_;
}