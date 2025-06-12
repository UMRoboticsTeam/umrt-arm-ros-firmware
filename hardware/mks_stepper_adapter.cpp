#include "umrt-arm-ros-firmware/mks_stepper_adapter.hpp"
#include <boost/bimap.hpp>

constexpr uint8_t NORM_FACTOR = 16;

MksStepperAdapter::MksStepperAdapter(const std::string& can_interface, const std::vector<uint16_t>& motor_ids, const std::chrono::duration<int64_t, std::milli>& query_period) : StepperAdapter(motor_ids.size()) {
    // Preprocess motor IDs into bimap we can use to convert between joint index and motor, and an unordered_set
    //     that MksController can use for its packet address lookups
    auto motor_ids_for_controller = std::make_unique<std::unordered_set<uint16_t>>(motor_ids.cbegin(), motor_ids.cend());
    this->motor_ids = std::make_unique<boost::bimap<uint16_t, uint16_t>>();
    for (size_t i = 0; i < motor_ids.size(); ++i) {
        this->motor_ids->insert(boost::bimap<uint16_t, uint16_t>::value_type(i, motor_ids.at(i)));
    }
    controller = std::make_unique<MksStepperController>(can_interface, std::move(motor_ids_for_controller), NORM_FACTOR);

    // Register to receive callbacks for responses to getPosition and getSpeed
    // Note: These callbacks will occur in another thread, so they need to be processed carefully
    this->controller->EGetPosition.connect([this](const uint16_t motor, const int32_t pos) -> void { this->updatePosition(this->motor_ids->right.at(motor), pos); });

    // Start the polling loops for message handling and joint state querying
    this->continue_polling = true;
    this->polling_thread = std::thread([this]() -> void { this->poll(); });
    this->querying_thread = std::thread([this, query_period]() -> void { this->queryPoll(query_period); });
}

MksStepperAdapter::~MksStepperAdapter() {
    if (this->continue_polling) {
        this->continue_polling = false;
        this->polling_thread.join();
        this->querying_thread.join();
    }
}

void MksStepperAdapter::connect(const std::string device, const int baud_rate) {}

void MksStepperAdapter::disconnect() {}

void MksStepperAdapter::setValues() {
    for (auto i = 0u; i < commands.size(); ++i) {
        // Note that the MksStepperController speed is in units of RPM (since we're using interpolated normalisation)
        this->controller->setSpeed(motor_ids->left.at(i), static_cast<int16_t>(std::round(this->commands.at(i))));
    }

    // TODO: Add gripper support
}

void MksStepperAdapter::poll() {
    // Run update loop approximately forever
    // TODO: Look into a better way of doing the polling loop which isn't so intensive
    while (continue_polling) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        this->controller->update();
    }
}

void MksStepperAdapter::queryController() {
    for (auto i = 0u; i < commands.size(); ++i) {
        this->controller->getPosition(this->motor_ids->left.at(i));
    }
}

void MksStepperAdapter::queryPoll(const std::chrono::milliseconds& period) {
    while (this->continue_polling) {
        std::this_thread::sleep_for(period);
        this->queryController();
    }
}