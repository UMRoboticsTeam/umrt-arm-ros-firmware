#include "umrt-arm-ros-firmware/StepperAdapter.hpp"

StepperAdapter::StepperAdapter(const std::size_t NUM_JOINTS) {
    // Set the size of the vectors
    // See note in header about how important it is for these to not change
    this->positions.resize(NUM_JOINTS);
    this->velocities.resize(NUM_JOINTS);
    this->commands.resize(NUM_JOINTS);
    this->positions_buffer.resize(NUM_JOINTS);
    this->velocities_buffer.resize(NUM_JOINTS);
}

StepperAdapter::~StepperAdapter() {}

double& StepperAdapter::getPositionRef(size_t index) {
    return this->positions[index];
}

double& StepperAdapter::getVelocityRef(std::size_t index) {
    return this->velocities[index];
}

double& StepperAdapter::getGripperPositionRef() {
    return this->gripper_position;
}

double& StepperAdapter::getCommandRef(std::size_t index) {
    return this->commands[index];
}

double& StepperAdapter::getGripperPositionCommandRef() {
    return this->cmd_gripper_pos;
}
