#pragma once

#include "pid.hpp"

class ErrorPidController
{
public:
    ErrorPidController(
        double kp,
        double ki,
        double kd);

    double update(
        double error,
        double dt);

    void reset();

    Pid& pid();

private:
    Pid pid_;
};