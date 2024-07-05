#ifndef STEPPER_ADAPTER_HPP
#define STEPPER_ADAPTER_HPP

#include <cstddef>
#include <exception>
#include <array>

#include "StepperController.h"

class StepperAdapter {
public:
    void init(std::size_t NUM_JOINTS);

    void connect(const std::string device, const int baud_rate);

    void disconnect();

    void setValues();

    void readValues();

    double& getPositionRef(std::size_t index);

    double& getVelocityRef(std::size_t index);

    double& getCommandRef(std::size_t index);

protected:
    StepperController controller;

    // It is very important that the size of these vectors is not changed after init has been called, since we need
    // element pointer stability. Unfortunately, we also need element mutability so const vectors can't be used. As
    // well, the size cannot be determined at compile-time so std::arrays can't be used either. Random-access is
    // required, so std::lists are not a good option either.
    std::vector<double> commands;
    std::vector<double> positions;
    std::vector<double> velocities;

    void initializedCheck();

private:
    bool initialized = false;
};

#endif // STEPPER_ADAPTER_HPP