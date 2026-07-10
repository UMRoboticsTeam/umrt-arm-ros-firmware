#include "umrt-arm-ros-firmware/wheel_adapter.hpp"
#include <boost/bimap.hpp>

WheelAdapter::WheelAdapter(const std::size_t num_joints) {

    this->positions.resize(num_joints);
    this->velocities.resize(num_joints);
    this->commands.resize(num_joints);

}

WheelAdapter::~WheelAdapter() {
}

void WheelAdapter::connect(const std::string device, const int baud_rate) {
    return;
}

void WheelAdapter::disconnect() {
    return;
}

void WheelAdapter::writeValues() {
    return;
}

void WheelAdapter::readValues() {
    return;
}

double& WheelAdapter::getPositionRef(const size_t index) {
    return this->positions[index];
}

double& WheelAdapter::getCommandRef(const std::size_t index) {
    return this->commands[index];
}

double& WheelAdapter::getVelocityRef(const std::size_t index) {
    return this->velocities[index];
}
