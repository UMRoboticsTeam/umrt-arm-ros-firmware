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

StepperAdapter::~StepperAdapter() = default;

void StepperAdapter::readValues() {
    // Acquire the locks for positions_buffer and velocities_buffer, and transfer the values
    {
        std::scoped_lock lock(this->positions_buffer_mx, this->velocities_buffer_mx);

        // Remember that we can't invalidate references, so we need to manually copy values
        for (auto i = 0u; i < commands.size(); ++i) {
            this->positions[i] = this->positions_buffer[i];
            this->velocities[i] = this->velocities_buffer[i];
        }
    }

    // By default, simply copy gripper position
    this->gripper_position = this->cmd_gripper_pos;
}

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

// Reminder: This is eligible to happen in another thread
void StepperAdapter::updatePosition(const uint8_t joint, const int32_t position) {
    // Acquire the lock for positions_buffer and write the new value
    {
        std::scoped_lock lock(this->positions_buffer_mx);
        this->positions_buffer[joint] = position;
    }
}

// Reminder: This is eligible to happen in another thread
void StepperAdapter::updateVelocity(const uint8_t joint, const int16_t speed) {
    // Acquire the lock for velocities_buffer and write the new value
    {
        std::scoped_lock lock(this->velocities_buffer_mx);
        this->velocities_buffer[joint] = speed;
    }
}
