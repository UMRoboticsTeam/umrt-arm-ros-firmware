#include <cmath>

#include "umrt-arm-ros-firmware/project_perry_controller.hpp"

constexpr uint8_t NORM_FACTOR = 16;
constexpr double STEPS_PER_REV = 200.0;

ProjectPerryController::ProjectPerryController(
        const std::string& can_interface, const std::vector<JointInfo>& joint_infos, const bool position_commandable,
        const double default_speed, const std::chrono::duration<int64_t, std::milli>& query_period, rclcpp::Logger& logger
)
    : StepperAdapter(joint_infos.size()), position_commandable(position_commandable), default_speed(default_speed),
      logger(logger) {
    // Preprocess motor IDs into bimap we can use to convert between joint index and motor, and an unordered_set
    //     that MksController can use for its packet address lookups

    auto motor_ids_for_controller = std::make_unique<std::unordered_set<uint16_t>>(joint_infos.size());
    this->motor_ids = std::make_unique<boost::bimap<uint16_t, uint16_t>>();
    this->reductions = std::make_unique<std::unordered_map<uint16_t, double>>();
    this->last_motor_commands = std::make_unique<std::unordered_map<uint16_t, int32_t>>();
    for (size_t i = 0; i < joint_infos.size(); ++i) {
        const JointInfo& j = joint_infos.at(i);
        motor_ids_for_controller->insert(j.motor_id);
        this->motor_ids->insert(boost::bimap<uint16_t, uint16_t>::value_type(i, j.motor_id));
        this->reductions->emplace(j.motor_id, j.reduction_factor);
        this->last_motor_commands->emplace(j.motor_id, 0);

        // TODO: Remove, only for testing with 1 motor
        this->updatePosition(i, 0);
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

ProjectPerryController::~ProjectPerryController() {
    if (this->continue_polling) {
        this->continue_polling = false;
        this->polling_thread.join();
        this->querying_thread.join();
    }
}

void ProjectPerryController::connect(const std::string device, const int baud_rate) {}

void ProjectPerryController::disconnect() {}

void ProjectPerryController::setValues() {
    if (this->position_commandable) {
        for (auto i = 0u; i < NUM_JOINTS; ++i) {
            auto const motor_id = this->motor_ids->left.at(i); // Convert joint ID to motor ID
            const auto reduction = this->reductions->at(motor_id);

            // Note that the MksStepperController speed is in units of RPM (since we're using interpolated normalisation)
            const auto position =
                    static_cast<int32_t>(std::round(this->position_commands.at(i) * reduction * STEPS_PER_REV / 2 / M_PI));
            auto speed = static_cast<int16_t>(std::round(this->velocity_commands.at(i) * reduction));
            if (speed == 0) { speed = static_cast<int16_t>(std::round(this->default_speed * reduction)); }

            // If this is a new command, log it (if in debug mode)
            if (this->last_motor_commands->at(motor_id) != position) {
                this->last_motor_commands->at(motor_id) = position;
                RCLCPP_DEBUG(this->logger, "Joint %d: Seeking to %d at %d", i, position, speed);
            }

            this->controller->seekPosition(motor_id, position, speed);

            // TODO: For now just copy commanded velocity into velocity feedback
            this->updateVelocity(i, this->velocity_commands.at(i));

            // TODO: Idea for closed loop control: We should monitor SEEK_POS responses, and once we get a "COMPLETED" if
            //       there is error from target position we send some more steps
        }
    } else {
        for (auto i = 0u; i < NUM_JOINTS; ++i) {
            // Note that the MksStepperController speed is in units of RPM (since we're using interpolated normalisation)
            this->controller->setSpeed(
                    motor_ids->left.at(i), static_cast<int16_t>(std::round(this->velocity_commands.at(i)))
            );
        }
    }

    // TODO: Add gripper support
}

void ProjectPerryController::poll() {
    // Run update loop approximately forever
    // TODO: Look into a better way of doing the polling loop which isn't so intensive
    while (continue_polling) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        this->controller->update(std::chrono::milliseconds(10));
    }
}

void ProjectPerryController::queryController() {
    for (auto i = 0u; i < NUM_JOINTS; ++i) { this->controller->getPosition(this->motor_ids->left.at(i)); }
}

void ProjectPerryController::queryPoll(const std::chrono::milliseconds& period) {
    while (this->continue_polling) {
        std::this_thread::sleep_for(period);
        this->queryController();
    }
}