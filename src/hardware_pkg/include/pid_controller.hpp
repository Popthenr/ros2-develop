#pragma once

#include "pid.hpp"

class PidController
{
public:
    PidController(
        double kp,
        double ki,
        double kd);

    double update(
        double target,
        double actual,
        double dt);

    void reset();

    Pid& pid();

private:
    Pid pid_;
};