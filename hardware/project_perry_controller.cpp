#include <cmath>

#include "umrt-arm-ros-firmware/project_perry_controller.hpp"

inline constexpr uint8_t NORM_FACTOR = 16;
inline constexpr double STEPS_PER_REV = 200.0;

inline constexpr size_t EXPECTED_JOINTS = 5;
inline constexpr size_t WRIST_PITCH_INDEX = 3;
inline constexpr size_t WRIST_ROLL_INDEX = 4;
inline constexpr size_t NON_DIFFERENTIAL_JOINTS[] = { 0, 1, 2 };

namespace {
    void validate_joints(const std::vector<StepperAdapter::JointInfo>& joint_infos, rclcpp::Logger& logger);
}

ProjectPerryController::ProjectPerryController(
        const std::string& can_interface, const std::vector<JointInfo>& joint_infos, const uint16_t gripper_id,
        const double default_speed, const std::chrono::duration<int64_t, std::milli>& query_period, rclcpp::Logger& logger
)
    : StepperAdapter(joint_infos.size()), default_speed(default_speed), logger(logger) {
    validate_joints(joint_infos, logger);

    // Preprocess motor IDs into bimap we can use to convert between joint index and motor, and an unordered_set
    //     that MksController can use for its packet address lookups
    // As well, do that for encoder IDs for applicable joints

    auto motor_ids_for_controller = std::make_unique<std::unordered_set<uint16_t>>(joint_infos.size());
    auto encoder_ids_for_interface = std::make_unique<std::unordered_set<uint32_t>>(joint_infos.size());
    this->motor_ids = std::make_unique<boost::bimap<uint16_t, uint16_t>>();
    this->encoder_ids = std::make_unique<boost::bimap<uint16_t, uint16_t>>();
    this->reductions = std::make_unique<std::unordered_map<uint16_t, double>>();
    this->last_motor_commands = std::make_unique<std::unordered_map<uint16_t, int32_t>>();
    this->encoder_initial_positions = std::make_unique<std::unordered_map<uint16_t, double>>();
    for (size_t i = 0; i < joint_infos.size(); ++i) {
        const JointInfo& j = joint_infos.at(i);
        motor_ids_for_controller->insert(j.motor_id);
        this->motor_ids->insert(boost::bimap<uint16_t, uint16_t>::value_type(i, j.motor_id));
        this->reductions->emplace(i, j.reduction_factor);
        this->last_motor_commands->emplace(i, 0);
        if (j.encoder_id != 0) {
            encoder_ids_for_interface->insert(j.encoder_id);
            this->encoder_initial_positions->emplace(i, NAN);
            this->encoder_ids->insert(boost::bimap<uint16_t, uint16_t>::value_type(i, j.encoder_id));
        }
    }
    this->controller =
            std::make_unique<MksStepperController>(can_interface, std::move(motor_ids_for_controller), NORM_FACTOR);
    this->encoders = std::make_unique<EncoderInterface>(can_interface, std::move(encoder_ids_for_interface));
    this->gripper = std::make_unique<ServoController>(can_interface, gripper_id);

    // Register to receive callbacks for responses to getPosition and getSpeed
    // Note: These callbacks will occur in another thread, so they need to be processed carefully
    this->controller->EGetPosition.connect([this](const uint16_t motor, const int32_t pos) -> void {
        const auto joint = this->motor_ids->right.at(motor);
        // If we have an encoder for this motor, skip motor feedback
        if (this->encoder_ids->left.find(joint) != this->encoder_ids->left.end()) { return; }

        // [rad] = [steps] / [steps / rev] * [2 pi rad / rev]
        // Also reduction factor
        auto position = pos / this->reductions->at(joint) / STEPS_PER_REV * 2 * M_PI;
        RCLCPP_DEBUG(this->logger, "Joint %u: MTR(id=%u) mtr position=%f (rad)", joint, motor, position);
        this->updatePosition(joint, position);
    });

    // Register for encoder callbacks
    this->encoders->angle_signal_raw.connect(
            [this](uint32_t encoder, uint16_t angle, uint16_t angular_vel, int16_t n_rotations) -> void {
                // [rad] = [15-bit position] / [2^15] * [2 pi rad / rev]
                // Also number of rotations, and reduction factor

                auto joint = this->encoder_ids->right.at(encoder);
                auto position = (angle / 32768.0 + n_rotations) * 2 * M_PI;
                if (std::isnan(this->encoder_initial_positions.get()->at(joint))) {
                    this->encoder_initial_positions.get()->at(joint) = position;
                    RCLCPP_INFO(
                            this->logger, "Joint %u: ENC(id=%u) initial encoder position = %f (rad), %f (deg)", joint,
                            encoder, position, position * 180.0 / M_PI
                    );
                }
                RCLCPP_DEBUG(
                        this->logger, "Joint %u: ENC(id=%u) mtr position=%f (rad), %f (deg)", joint, encoder, position,
                        position * 180.0 / M_PI
                );
                this->updatePosition(joint, position);
            }
    );

    // Start the polling loops for message handling and joint state querying
    this->continue_polling = true;
    this->polling_thread = std::thread([this]() -> void { this->poll(); });
    this->querying_thread = std::thread([this, query_period]() -> void { this->queryPoll(query_period); });
    this->encoders_thread = std::thread([this]() -> void { this->encoders.get()->begin_read_loop(); });
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
    double position_commands_remapped[EXPECTED_JOINTS] = {0};
    double velocity_commands_remapped[EXPECTED_JOINTS] = {0};

    for (const auto j : NON_DIFFERENTIAL_JOINTS) {
        position_commands_remapped[j] = this->position_commands.at(j);
        velocity_commands_remapped[j] = this->velocity_commands.at(j);
    }

    // Account for differential wrist
    auto wrist_pitch_pos_cmd = this->position_commands.at(WRIST_PITCH_INDEX);
    auto wrist_roll_pos_cmd = this->position_commands.at(WRIST_ROLL_INDEX);
    
    auto wrist_pitch_vel_cmd = this->velocity_commands.at(WRIST_PITCH_INDEX);
    auto wrist_roll_vel_cmd = this->velocity_commands.at(WRIST_ROLL_INDEX);
    
    // NOTE: This doesn't consider the reduction from the differential gears
    // to the rolling portion of the wrist
    position_commands_remapped[WRIST_PITCH_INDEX] = wrist_pitch_pos_cmd + (wrist_roll_pos_cmd / 2);
    position_commands_remapped[WRIST_ROLL_INDEX] = wrist_pitch_pos_cmd - (wrist_roll_pos_cmd / 2);

    velocity_commands_remapped[WRIST_PITCH_INDEX] = (wrist_pitch_vel_cmd + wrist_roll_vel_cmd) / 2;
    velocity_commands_remapped[WRIST_ROLL_INDEX] = (wrist_pitch_vel_cmd + wrist_roll_vel_cmd) / 2;

    for (int j = 0; j < EXPECTED_JOINTS; j++) {
        const auto motor_id = this->motor_ids->left.at(j); // Convert joint ID to motor ID
        const auto encoder_id = this->encoder_ids.get()->left.at(j);
        const auto reduction = this->reductions->at(j);

        double target_position_rad = position_commands_remapped[j];

        // Correct for encoders, if present.
        if (encoder_id != 0) {
            auto offset = this->encoder_initial_positions.get()->at(j);
            if (std::isnan(offset)) {
                auto clk = rclcpp::Clock();
                RCLCPP_WARN_THROTTLE(
                        this->logger, clk, 10000,
                        "Joint %u: Tried to send a position to a motor(id=%u) with a configured encoder(id=%u) before the "
                        "encoder's first position response.\nCannot determine the encoder offset. No action will be taken. "
                        "If multiple motors have this issue, only one will show in the logs.",
                        j, motor_id, encoder_id
                );
                continue;
            }
            RCLCPP_DEBUG(this->logger, "Joint %u: target_position_rad=%f", j, target_position_rad);
            target_position_rad -= offset;
            RCLCPP_DEBUG(this->logger, "Joint %u: offset target_position_rad=%f", j, target_position_rad);
        }

        // Note that the MksStepperController speed is in units of RPM (since we're using interpolated normalisation)
        auto target_position_steps =
                static_cast<int32_t>(std::round(target_position_rad * reduction * STEPS_PER_REV / 2 / M_PI));
        auto speed = static_cast<int16_t>(std::round(velocity_commands_remapped[j] * reduction));
        if (speed == 0) { speed = static_cast<int16_t>(std::round(this->default_speed * reduction)); }

        RCLCPP_DEBUG(this->logger, "Joint %u: target_position_steps=%i", j, target_position_steps);

        // If this is a new command, log it (if in debug mode)
        if (this->last_motor_commands->at(j) != target_position_steps) {
            this->last_motor_commands->at(j) = target_position_steps;
            RCLCPP_DEBUG(this->logger, "Joint %lu: Seeking to %d at %d", j, target_position_steps, speed);
            this->controller->seekPosition(motor_id, target_position_steps, speed);
        }
        // TODO: Consider only sending commands to the controller if they are new, and sending a stop beforehand so the
        //       previous target is overridden. Might make more sense to do on MksController side.

        // TODO: For now just copy commanded velocity into velocity feedback
        this->updateVelocity(j, this->velocity_commands.at(j));

        // TODO: Idea for closed loop control: We should monitor SEEK_POS responses, and once we get a "COMPLETED" if
        //       there is error from target position we send some more steps
    }

    // Note that 255 is exactly representable in IEEE754 double
    this->gripper->send(static_cast<uint8_t>(std::round(std::clamp(gripper_position, 0.0, 255.0))));
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
    for (auto j = 0u; j < NUM_JOINTS; ++j) {
        // Only query controllers which we don't have encoders for
        if (this->encoder_ids->left.find(j) == this->encoder_ids->left.end()) {
            this->controller->getPosition(this->motor_ids->left.at(j));
        }
    }
}

void ProjectPerryController::queryPoll(const std::chrono::milliseconds& period) {
    while (this->continue_polling) {
        std::this_thread::sleep_for(period);
        this->queryController();
    }
}

namespace {
    /**
     * Asserts that the provided joint configuration is valid for a Project Perry system.
     * @param joint_infos joint information
     * @param logger ROS logger to use
     * @throws std::runtime_error if something wrong is found with the configuration
     */
    void validate_joints(const std::vector<StepperAdapter::JointInfo>& joint_infos, rclcpp::Logger& logger) {
        bool valid = true;

        // Check we have right number of joints
        if (joint_infos.size() != EXPECTED_JOINTS) {
            RCLCPP_FATAL(
                    logger,
                    "Xacro configuration not valid for ProjectPerryController: Wrong number of joints (%lu instead of %lu)",
                    joint_infos.size(), EXPECTED_JOINTS
            );
            // We're going to do some index-based checks after, so we need to throw here if this isn't correct instead of
            // accumulating errors
            throw std::runtime_error("Xacro configuration not valid for ProjectPerryController; see ROS log");
        }

        // Check differential wrist joints are labeled (so that we know indices are correct since xacro doesn't enforce joint order)
        if (!joint_infos.at(WRIST_PITCH_INDEX).differential) {
            RCLCPP_FATAL(
                    logger,
                    "Xacro configuration not valid for ProjectPerryController: Joint at index %lu not labeled as "
                    "differential (are your joints in the correct order in the xacro?)",
                    WRIST_PITCH_INDEX
            );
            valid = false;
        }
        if (!joint_infos.at(WRIST_ROLL_INDEX).differential) {
            RCLCPP_FATAL(
                    logger,
                    "Xacro configuration not valid for ProjectPerryController: Joint at index %lu not labeled as "
                    "differential (are your joints in the correct order in the xacro?)",
                    WRIST_PITCH_INDEX
            );
            valid = false;
        }

        // Check that all other joints are not labeled differential
        for (const auto i : NON_DIFFERENTIAL_JOINTS) {
            if (joint_infos.at(i).differential) {
                RCLCPP_FATAL(
                        logger,
                        "Xacro configuration not valid for ProjectPerryController: Joint at index %lu was unexpectedly "
                        "labeled as differential (are your joints in the correct order in the xacro?)",
                        i
                );
                valid = false;
            }
        }

        // Check differential wrist joints have same reduction ratios
        if (joint_infos.at(WRIST_PITCH_INDEX).reduction_factor != joint_infos.at(WRIST_ROLL_INDEX).reduction_factor) {
            RCLCPP_FATAL(
                    logger, "Xacro configuration not valid for ProjectPerryController: Different reduction ratio for "
                            "differential wrist"
            );
            valid = false;
        }

        if (!valid) { throw std::runtime_error("Xacro configuration not valid for ProjectPerryController; see ROS log"); }
    }
} // namespace