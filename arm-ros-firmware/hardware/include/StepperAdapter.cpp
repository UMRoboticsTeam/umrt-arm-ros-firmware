//
// Created by Noah on 2024-07-05.
//

#include "StepperAdapter.hpp"


void StepperAdapter::init(std::size_t NUM_JOINTS) {
    this->positions.resize(NUM_JOINTS);
    this->velocities.resize(NUM_JOINTS);
    this->commands.resize(NUM_JOINTS);
    this->initialized = true;
}

void StepperAdapter::connect(const std::string device, const int baud_rate) {
    initializedCheck();
    this->controller.connect(device, baud_rate);
}

void StepperAdapter::disconnect() {
    initializedCheck();
    this->controller.disconnect();
}

double& StepperAdapter::getPositionRef(size_t index) {
    initializedCheck();
    return this->positions[index];
}

double& StepperAdapter::getVelocityRef(std::size_t index) {
    initializedCheck();
    return this->velocities[index];
}

double& StepperAdapter::getCommandRef(std::size_t index) {
    initializedCheck();
    return this->commands[index];
}

void StepperAdapter::setValues() {
    initializedCheck();
    for (auto i = 0u; i < commands.size(); ++i) {
        this->controller.setSpeed(i, (int16_t)std::round(10*this->commands[i]));
    }
}

void StepperAdapter::readValues(){
    for (auto i = 0u; i < commands.size(); ++i) {
        // TODO: Update positions/velocity vectors with info with GET_SPEED etc. requests through controller
        this->positions[i] = this->commands[i];
        this->velocities[i] = this->commands[i];
    }
}

void StepperAdapter::initializedCheck() {
    if (!this->initialized) { throw std::logic_error("StepperAdapter not initialized"); }
}