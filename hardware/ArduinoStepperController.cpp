#include "umrt-arm-ros-firmware/ArduinoStepperAdapter.hpp"
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

constexpr boost::log::trivial::severity_level LOG_LEVEL = boost::log::trivial::debug;

ArduinoStepperAdapter::~ArduinoStepperAdapter() {
    if (this->initialized) {
        this->disconnect();
    }
}

void ArduinoStepperAdapter::init(
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

    // Register to receive callbacks for responses to getPosition and getSpeed
    // Note: These callbacks will occur in another thread, so they need to be processed carefully
    this->controller.EGetPosition.connect([this](uint8_t joint, int32_t pos) -> void { this->updatePosition(joint, pos); });
    this->controller.EGetSpeed.connect([this](uint8_t joint, int16_t speed) -> void { this->updateVelocity(joint, speed); });

    // Start the polling loops for message handling and joint state querying
    this->continue_polling = true;
    this->polling_thread = std::thread([this]() -> void { this->poll(); });
    this->querying_thread = std::thread([this, query_period]() -> void { this->queryPoll(query_period); });

    this->initialized = true;
}

void ArduinoStepperAdapter::connect(const std::string device, const int baud_rate) {
    initializedCheck();
    this->controller.connect(device, baud_rate);
}

void ArduinoStepperAdapter::disconnect() {
    initializedCheck();
    this->continue_polling = false;
    this->controller.disconnect();
}


void ArduinoStepperAdapter::setValues() {
    initializedCheck();
    for (auto i = 0u; i < commands.size(); ++i) {
        // Note that the StepperController speed is specified in units of in 1/10 RPM
        this->controller.setSpeed(i, (int16_t)std::round(10 * this->commands[i]));
    }

    this->controller.setGripper((uint8_t)std::round(this->cmd_gripper_pos));
}

void ArduinoStepperAdapter::readValues() {
    // Acquire the locks for positions_buffer and velocities_buffer, and transfer the values
    {
        std::scoped_lock lock(this->positions_buffer_mx, this->velocities_buffer_mx);

        // Remember that we can't invalidate references, so we need to manually copy values
        for (auto i = 0u; i < commands.size(); ++i) {
            this->positions[i] = this->positions_buffer[i];
            this->velocities[i] = this->velocities_buffer[i];
        }
    }

    // Simply copy gripper position
    this->gripper_position = this->cmd_gripper_pos;
}

void ArduinoStepperAdapter::poll() {
    // Run update loop approximately forever
    // TODO: Look into a better way of doing the polling loop which isn't so intensive
    while (continue_polling) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        this->controller.update();
    }
}

void ArduinoStepperAdapter::queryController() {
    for (auto i = 0u; i < commands.size(); ++i) {
        this->controller.getPosition(i);
        this->controller.getSpeed(i);
    }
}

// Reminder: This happens in another thread
void ArduinoStepperAdapter::updatePosition(const uint8_t joint, const int32_t position) {
    // Acquire the lock for positions_buffer and write the new value
    {
        std::scoped_lock lock(this->positions_buffer_mx);
        this->positions_buffer[joint] = position;
    }
}

// Reminder: This happens in another thread
void ArduinoStepperAdapter::updateVelocity(const uint8_t joint, const int16_t speed) {
    // Acquire the lock for velocities_buffer and write the new value
    {
        std::scoped_lock lock(this->velocities_buffer_mx);
        this->velocities_buffer[joint] = speed;
    }
}

void ArduinoStepperAdapter::initializedCheck() {
    if (!this->initialized) { throw std::logic_error("ArduinoStepperAdapter not initialized"); }
}

void ArduinoStepperAdapter::queryPoll(const std::chrono::milliseconds& period) {
    while (this->continue_polling) {
        std::this_thread::sleep_for(period);
        this->queryController();
    }
}