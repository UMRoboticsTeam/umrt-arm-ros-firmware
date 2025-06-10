#include "umrt-arm-ros-firmware/mks_stepper_adapter.hpp"

MksStepperAdapter::MksStepperAdapter(const std::size_t NUM_JOINTS, const std::chrono::duration<int64_t, std::milli>& query_period) : StepperAdapter(NUM_JOINTS) {
    // Register to receive callbacks for responses to getPosition and getSpeed
    // Note: These callbacks will occur in another thread, so they need to be processed carefully
    this->controller.EGetPosition.connect([this](const uint8_t joint, const int32_t pos) -> void { this->updatePosition(joint, pos); });
    this->controller.EGetSpeed.connect([this](const uint8_t joint, const int16_t speed) -> void { this->updateVelocity(joint, speed); });

    // Start the polling loops for message handling and joint state querying
    this->continue_polling = true;
    this->polling_thread = std::thread([this]() -> void { this->poll(); });
    this->querying_thread = std::thread([this, query_period]() -> void { this->queryPoll(query_period); });
}

MksStepperAdapter::~MksStepperAdapter() {
    if (true) { // TODO: Figure out how to tell if disconnect need to be called
        MksStepperAdapter::disconnect();
    }
}

void MksStepperAdapter::connect(const std::string device, const int baud_rate) {
    this->controller.connect(device, baud_rate);
}

void MksStepperAdapter::disconnect() {
    this->continue_polling = false;
    this->controller.disconnect();
}


void MksStepperAdapter::setValues() {
    for (auto i = 0u; i < commands.size(); ++i) {
        // Note that the StepperController speed is specified in units of in 1/10 RPM
        this->controller.setSpeed(i, static_cast<int16_t>(std::round(10 * this->commands[i])));
    }

    this->controller.setGripper(static_cast<uint8_t>(std::round(this->cmd_gripper_pos)));
}

void MksStepperAdapter::poll() {
    // Run update loop approximately forever
    // TODO: Look into a better way of doing the polling loop which isn't so intensive
    while (continue_polling) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        this->controller.update();
    }
}

void MksStepperAdapter::queryController() {
    for (auto i = 0u; i < commands.size(); ++i) {
        this->controller.getPosition(i);
        this->controller.getSpeed(i);
    }
}

void MksStepperAdapter::queryPoll(const std::chrono::milliseconds& period) {
    while (this->continue_polling) {
        std::this_thread::sleep_for(period);
        this->queryController();
    }
}