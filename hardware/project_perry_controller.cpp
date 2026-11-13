#include <cmath>

#include "umrt-arm-ros-firmware/project_perry_controller.hpp"

inline constexpr uint8_t NORM_FACTOR = 16;
inline constexpr double STEPS_PER_REV = 200.0;

inline constexpr size_t EXPECTED_JOINTS = 5;
inline constexpr size_t WRIST_PITCH_INDEX = 3;
inline constexpr size_t WRIST_ROLL_INDEX = 4;
inline constexpr size_t NON_DIFFERENTIAL_JOINTS[] = { 0, 1, 2 };

// Note that this value is outside of the range of int32_t used to store number of steps for
// the motors, so the motor position will never naturally be set to this value.
inline constexpr int64_t MOTOR_POSITION_UNSET = static_cast<int64_t>(std::numeric_limits<uint32_t>::max()) + 1;

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
    this->motor_initial_positions = std::make_unique<std::unordered_map<uint16_t, int64_t>>();
    for (size_t i = 0; i < joint_infos.size(); ++i) {
        const JointInfo& j = joint_infos.at(i);
        RCLCPP_INFO(this->logger, "Joint %ld: Registering motor id %d", i, j.motor_id);
        motor_ids_for_controller->insert(j.motor_id);
        this->motor_ids->insert(boost::bimap<uint16_t, uint16_t>::value_type(i, j.motor_id));
        this->reductions->emplace(i, j.reduction_factor);
        this->last_motor_commands->emplace(i, 0);
        if (j.encoder_id == 0) {
            // Since there is no encoder to ensure absolute positions in the first place, these offsets are not useful.
            this->motor_initial_positions->emplace(i, 0);
            this->encoder_initial_positions->emplace(i, 0.0);
        } else {
            RCLCPP_INFO(this->logger, "Joint %ld: Registering encoder id %d", i, j.encoder_id);
            encoder_ids_for_interface->insert(j.encoder_id);
            this->motor_initial_positions->emplace(i, MOTOR_POSITION_UNSET);
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
        if (this->encoder_ids->left.find(joint) != this->encoder_ids->left.end()) {
            if (this->motor_initial_positions->at(joint) == MOTOR_POSITION_UNSET) {
                double position = pos / this->reductions->at(joint) / STEPS_PER_REV * 2 * M_PI;
                this->motor_initial_positions->at(joint) = (int64_t)pos;
                RCLCPP_INFO(
                    this->logger, "Joint %u: MTR(id=%u) initial motor position = %d (steps), %f (rad), %f (deg)", joint,
                    motor, pos, position, position * 180.0 / M_PI
                );
            }
            return;
        }

        // [rad] = [steps] / [steps / rev] * [2 pi rad / rev]
        // Also reduction factor
        double position = pos / this->reductions->at(joint) / STEPS_PER_REV * 2 * M_PI;
        RCLCPP_DEBUG(this->logger, "Joint %u: MTR(id=%u) mtr position=%f (rad)", joint, motor, position);
        this->updatePosition(joint, position);
    });

    // Register for encoder callbacks
    this->encoders->angle_signal.connect(
            [this](uint32_t encoder, double angle, double angular_vel, int16_t n_rotations) -> void {
                // [revolutions] = [deg] / [360.0] + n_rotations
                // [radians] = [revolutions] * 2PI

                uint16_t joint = this->encoder_ids->right.at(encoder);
                double position = (angle / 360.0 + n_rotations) * 2 * M_PI;
                if (std::isnan(this->encoder_initial_positions->at(joint))) {
                    this->encoder_initial_positions->at(joint) = position;
                    RCLCPP_INFO(
                            this->logger, "Joint %u: ENC(id=%u) initial encoder position = %f (rad), %f (deg)", joint,
                            encoder, position, position * 180.0 / M_PI
                    );
                }
                auto clk = rclcpp::Clock();
                RCLCPP_DEBUG_THROTTLE(
                        this->logger, clk, 2500, "Joint %u: ENC(id=%u) mtr position=%f (rad), %f (deg)", joint, encoder, position,
                        position * 180.0 / M_PI
                );
                this->updatePosition(joint, position);
            }
    );

    // Start the polling loops for message handling and joint state querying
    this->continue_polling = true;
    this->polling_thread = std::thread([this]() -> void { this->poll(); });
    this->querying_thread = std::thread([this, query_period]() -> void { this->queryPoll(query_period); });
    this->encoders_thread = std::thread([this]() -> void { this->encoders->begin_read_loop(); });
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
    double position_commands_remapped[EXPECTED_JOINTS] = { 0 };
    double velocity_commands_remapped[EXPECTED_JOINTS] = { 0 };

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

    for (int j = 0; j < (int)EXPECTED_JOINTS; j++) {
        const auto motor_id = this->motor_ids->left.at(j); // Convert joint ID to motor ID
        const auto encoder_id = this->encoder_ids->left.at(j);
        const auto reduction = this->reductions->at(j);

        double target_position_rad = position_commands_remapped[j];

        // Correct for encoders, if present.
        double offset_enc = 0.0;
        offset_enc = this->encoder_initial_positions->at(j);
        if (std::isnan(offset_enc)) {
            auto clk = rclcpp::Clock();
            RCLCPP_WARN_THROTTLE(
                this->logger, clk, 10000,
                    "Joint %u: NO_ENCODER_INITIAL_POS: Tried to send a position to a motor(id=%u) with a configured "
                    "encoder(id=%u) before the encoder's first position response.\nCannot determine the encoder offset. "
                    "No action will be taken. If multiple motors have this issue, only one will show in the logs.",
                    j, motor_id, encoder_id
            );
            continue;
        }
  
        int64_t offset_mtr = this->motor_initial_positions->at(j);
        if (offset_mtr == MOTOR_POSITION_UNSET) {
            auto clk = rclcpp::Clock();
            RCLCPP_WARN_THROTTLE(
                    this->logger, clk, 10000,
                    "Joint %u: NO_MOTOR_INITIAL_POS: Tried to send a position to a motor(id=%u) with a configured "
                    "encoder(id=%u) before knowing the motor's initial position.\nCannot determine the motor offset, making "
                    "absolute positioning useless. No action will be taken. If multiple motors have this issue, only one "
                    "will show in the logs.",
                    j, motor_id, encoder_id
            );
            continue;
        }

        // Offset on the reading side
        // RCLCPP_DEBUG(this->logger, "Joint %u: target_position_rad=%f", j, target_position_rad);
        target_position_rad -= offset_enc;
        // RCLCPP_DEBUG(this->logger, "Joint %u: offset target_position_rad=%f", j, target_position_rad);

        // Note that the MksStepperController speed is in units of RPM (since we're using interpolated normalisation)
        auto target_position_steps =
                static_cast<int32_t>(std::round(target_position_rad * reduction * STEPS_PER_REV / 2 / M_PI));
        auto speed = static_cast<int16_t>(std::round(velocity_commands_remapped[j] * reduction));
        if (speed == 0) { speed = static_cast<int16_t>(std::round(this->default_speed * reduction)); }
      

        // This offset will be non-zero only when the motor controllers power up and move
        // before the control software is enabled.
        target_position_steps += offset_mtr;
        // auto clk = rclcpp::Clock();
        // RCLCPP_DEBUG_THROTTLE(
        //     this->logger, clk, 2500,
        //     "Joint %u: target_position=%d (steps), %f (rad)", j, target_position_steps, position_commands_remapped[j]);
        // If this is a new command, log it (if in debug mode)
        if (this->last_motor_commands->at(j) != target_position_steps) {
            this->last_motor_commands->at(j) = target_position_steps;
            RCLCPP_DEBUG(this->logger, "Joint %u: target_position=%d (steps), %f (rad); speed=%d", j, target_position_steps, position_commands_remapped[j], speed);
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
        // and don't have the start position set.
        bool has_encoder = this->encoder_ids->left.find(j) != this->encoder_ids->left.end();
        bool needs_motor_initial_pos = this->motor_initial_positions->at(j) == MOTOR_POSITION_UNSET;
        if (!has_encoder || (has_encoder && needs_motor_initial_pos)) {
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