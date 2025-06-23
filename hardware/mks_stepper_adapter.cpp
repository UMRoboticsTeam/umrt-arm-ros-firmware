#include <cmath>

#include "umrt-arm-ros-firmware/mks_stepper_adapter.hpp"

constexpr uint8_t NORM_FACTOR = 16;
constexpr double STEPS_PER_REV = 200.0;

MksStepperAdapter::MksStepperAdapter(const std::string& can_interface, const std::vector<JointInfo>& joint_infos, const std::chrono::duration<int64_t, std::milli>& query_period) : StepperAdapter(motor_ids.size()) {
    // Preprocess motor IDs into bimap we can use to convert between joint index and motor, and an unordered_set
    //     that MksController can use for its packet address lookups

    auto motor_ids_for_controller = std::make_unique<std::unordered_set<uint16_t>>(joint_infos.size());
    this->motor_ids = std::make_unique<boost::bimap<uint16_t, uint16_t>>();
    this->reductions = std::make_unique<std::unordered_map<uint16_t, double>>();
    for (size_t i = 0; i < joint_infos.size(); ++i) {
        const JointInfo& j = joint_infos.at(i);
        motor_ids_for_controller->insert(j.motor_id);
        this->motor_ids->insert(boost::bimap<uint16_t, uint16_t>::value_type(i, j.motor_id));
    }
    controller = std::make_unique<MksStepperController>(can_interface, std::move(motor_ids_for_controller), NORM_FACTOR);

    // Register to receive callbacks for responses to getPosition and getSpeed
    // Note: These callbacks will occur in another thread, so they need to be processed carefully
    this->controller->EGetPosition.connect([this](const uint16_t motor, const int32_t pos) -> void {
        // [rad] = [steps] / [steps / rev] * [2 pi rad / rev]
        // Also reduction factor
        this->updatePosition(this->motor_ids->right.at(motor), pos / this->reductions->at(motor) / STEPS_PER_REV * 2 * M_PI);
    });

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
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        this->controller->update(std::chrono::milliseconds(10));
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