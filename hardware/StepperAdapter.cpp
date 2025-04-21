#include "umrt-arm-ros-firmware/StepperAdapter.hpp"
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>

constexpr boost::log::trivial::severity_level LOG_LEVEL = boost::log::trivial::debug;

StepperAdapter::StepperAdapter(
        const std::size_t NUM_JOINTS,
        const std::chrono::duration<int64_t, std::milli>& query_period
) {
    // Set the library's log level
    boost::log::core::get()->set_filter(boost::log::trivial::severity >= LOG_LEVEL);

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
