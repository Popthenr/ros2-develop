#include "errorpid_controller.hpp"

ErrorPidController::ErrorPidController(
    double kp,
    double ki,
    double kd)
    : pid_(kp, ki, kd)
{
}

double ErrorPidController::update(
    double error,
    double dt)
{
    return pid_.update(error, dt);
}

void ErrorPidController::reset()
{
    pid_.reset();
}

Pid& ErrorPidController::pid()
{
    return pid_;
}