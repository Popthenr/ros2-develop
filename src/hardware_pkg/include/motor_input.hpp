#pragma once

class MotorInput
{
public:
    virtual ~MotorInput() = default;

    virtual void set_command(double command) = 0;

    virtual double angle() const = 0;

    virtual double speed() const = 0;
};