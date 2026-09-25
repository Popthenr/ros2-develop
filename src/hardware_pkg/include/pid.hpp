#pragma once

#include <algorithm>

class Pid
{
public:
    Pid(double kp, double ki, double kd);

    double update(double error, double dt);

    void reset();

    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;

    double integral_min = -20.0;
    double integral_max = 20.0;

    double output_min = -20.0;
    double output_max = 20.0;

private:
    double integral_ = 0.0;
    double previous_error_ = 0.0;
};