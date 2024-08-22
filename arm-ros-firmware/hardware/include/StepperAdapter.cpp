#include "StepperAdapter.hpp"

#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/file.hpp>

constexpr boost::log::trivial::severity_level LOG_LEVEL = boost::log::trivial::debug;
constexpr uint32_t TOTAL_LOG_SIZE = 100 * 1024 * 1024; // 100 MiB

void StepperAdapter::init(const std::size_t NUM_JOINTS) {
    // Setup logging
    boost::log::add_file_log(
            boost::log::keywords::file_name = "[%TimeStamp%]_%N.log",
            boost::log::keywords::rotation_size = TOTAL_LOG_SIZE,
            boost::log::keywords::format = "[%TimeStamp%]: %Message%",
            boost::log::keywords::auto_flush = true
    );
    boost::log::core::get()->set_filter(boost::log::trivial::severity >= LOG_LEVEL);
    boost::log::add_common_attributes();
    BOOST_LOG_TRIVIAL(debug) << "Logging started";

    // Set the size of the vectors
    // See note in header about how important it is for these to not change
    this->positions.resize(NUM_JOINTS);
    this->velocities.resize(NUM_JOINTS);
    this->commands.resize(NUM_JOINTS);

    // Start the polling loop
    this->polling_thread = std::thread(&StepperAdapter::poll, this);

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

double& StepperAdapter::getGripperPositionRef() {
    initializedCheck();
    return this->gripper_position;
}

double& StepperAdapter::getCommandRef(std::size_t index) {
    initializedCheck();
    return this->commands[index];
}

double& StepperAdapter::getGripperPositionCommandRef() {
    initializedCheck();
    return this->cmd_gripper_pos;
}

void StepperAdapter::setValues() {
    initializedCheck();
    for (auto i = 0u; i < commands.size(); ++i) {
        // Note that the StepperController speed is specified in units of in 1/10 RPM
        this->controller.setSpeed(i, (int16_t)std::round(10 * this->commands[i]));
    }

    this->controller.setGripper((uint8_t)std::round(this->cmd_gripper_pos));
}

void StepperAdapter::readValues() {
    for (auto i = 0u; i < commands.size(); ++i) {
        // TODO: Update positions/velocity vectors with info with GET_SPEED etc. requests through controller
        this->positions[i] = this->commands[i];
        this->velocities[i] = this->commands[i];
    }

    this->gripper_position = this->cmd_gripper_pos;
}

[[noreturn]] void StepperAdapter::poll() {
    // Run update loop forever
    // TODO: Look into a better way of doing the polling loop which isn't so intensive
    for (;;) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        this->controller.update();
    }
}

void StepperAdapter::initializedCheck() {
    if (!this->initialized) { throw std::logic_error("StepperAdapter not initialized"); }
}