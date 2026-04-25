#include <cmath>

#include "umrt-arm-ros-firmware/project_perry_controller.hpp"

constexpr uint8_t NORM_FACTOR = 16;
constexpr double STEPS_PER_REV = 200.0;

constexpr size_t EXPECTED_JOINTS = 5;
constexpr size_t WRIST_PITCH_INDEX = 3;
constexpr size_t WRIST_ROLL_INDEX = 4;
const std::vector<size_t> NON_DIFFERENTIAL_JOINTS{ 0, 1, 2 };

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
    for (size_t i = 0; i < joint_infos.size(); ++i) {
        const JointInfo& j = joint_infos.at(i);
        motor_ids_for_controller->insert(j.motor_id);
        this->motor_ids->insert(boost::bimap<uint16_t, uint16_t>::value_type(i, j.motor_id));
        this->reductions->emplace(j.motor_id, j.reduction_factor);
        this->last_motor_commands->emplace(j.motor_id, 0);
        if (j.encoder_id != 0) {
            encoder_ids_for_interface->insert(j.encoder_id);
            this->motor_ids->insert(boost::bimap<uint16_t, uint16_t>::value_type(i, j.encoder_id));
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
        this->updatePosition(this->motor_ids->right.at(motor), pos / this->reductions->at(motor) / STEPS_PER_REV * 2 * M_PI);
    });

    // Register for encoder callbacks
    this->encoders->angle_signal.connect(
            [this](uint32_t encoder, uint16_t angle, uint16_t angular_vel, uint16_t n_rotations) -> void {
                // TODO: Workaround for bug in umrt-arm-encoder-driver - n_rotations is supposed to be signed
                n_rotations = static_cast<int16_t>(n_rotations);

                // [rad] = [15-bit position] / [2^15] * [2 pi rad / rev]
                // Also number of rotations, and reduction factor
                this->updatePosition(this->encoder_ids->right.at(encoder), (angle / 32768.0 + n_rotations) * 2 * M_PI);
            }
    );

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
    for (const auto j : NON_DIFFERENTIAL_JOINTS) {
        const auto motor_id = this->motor_ids->left.at(j); // Convert joint ID to motor ID
        const auto reduction = this->reductions->at(motor_id);

        // Note that the MksStepperController speed is in units of RPM (since we're using interpolated normalisation)
        const auto position =
                static_cast<int32_t>(std::round(this->position_commands.at(j) * reduction * STEPS_PER_REV / 2 / M_PI));
        auto speed = static_cast<int16_t>(std::round(this->velocity_commands.at(j) * reduction));
        if (speed == 0) { speed = static_cast<int16_t>(std::round(this->default_speed * reduction)); }

        // If this is a new command, log it (if in debug mode)
        if (this->last_motor_commands->at(motor_id) != position) {
            this->last_motor_commands->at(motor_id) = position;
            RCLCPP_DEBUG(this->logger, "Joint %lu: Seeking to %d at %d", j, position, speed);
        }
        this->controller->seekPosition(motor_id, position, speed);

        // TODO: For now just copy commanded velocity into velocity feedback
        this->updateVelocity(j, this->velocity_commands.at(j));

        // TODO: Idea for closed loop control: We should monitor SEEK_POS responses, and once we get a "COMPLETED" if
        //       there is error from target position we send some more steps
    }

    // Handle differential wrist
    // We define the wrist_pitch motor as the left motor, i.e. the one which moving forward produces negative pitch
    const auto left_motor_id = this->motor_ids->left.at(WRIST_PITCH_INDEX);
    const auto right_motor_id = this->motor_ids->left.at(WRIST_ROLL_INDEX);
    const auto reduction = this->reductions->at(left_motor_id); // Recall we assert reductions are the same

    // Kind of hacky, but we will use the average of the specified speeds
    auto speed = static_cast<int16_t>(std::round(
            (this->velocity_commands.at(WRIST_PITCH_INDEX) + this->velocity_commands.at(WRIST_ROLL_INDEX)) / 2 * reduction
    ));
    if (speed == 0) { speed = static_cast<int16_t>(std::round(this->default_speed * reduction)); }

    // Calculate the pseudo-joint targets in units of steps from the zero position
    const auto wrist_pitch_target = static_cast<int32_t>(
            std::round(this->position_commands.at(WRIST_PITCH_INDEX) * reduction * STEPS_PER_REV / 2 / M_PI)
    );
    const auto wrist_roll_target = static_cast<int32_t>(
            std::round(this->position_commands.at(WRIST_ROLL_INDEX) * reduction * STEPS_PER_REV / 2 / M_PI)
    );

    // Kinematics of a differential wrist:
    // Pitch is the average of the motor positions, and roll is the difference in motor positions
    // Our motor convention means that positive (CW when looking down arm) roll means that both motors are moving forwards
    //      (i.e. left motor producing negative pitch, right motor positive pitch)
    const auto left_motor_position = wrist_pitch_target + wrist_roll_target / 2;
    const auto right_motor_position = wrist_pitch_target - wrist_roll_target / 2;

    // If this is a new command, log it (if in debug mode)
    if (this->last_motor_commands->at(WRIST_PITCH_INDEX) != left_motor_position) {
        this->last_motor_commands->at(WRIST_PITCH_INDEX) = left_motor_position;
        RCLCPP_DEBUG(this->logger, "Joint %lu: Seeking to %d at %d", WRIST_PITCH_INDEX, left_motor_position, speed);
    }
    if (this->last_motor_commands->at(WRIST_ROLL_INDEX) != right_motor_position) {
        this->last_motor_commands->at(WRIST_ROLL_INDEX) = right_motor_position;
        RCLCPP_DEBUG(this->logger, "Joint %lu: Seeking to %d at %d", WRIST_ROLL_INDEX, right_motor_position, speed);
    }
    this->controller->seekPosition(left_motor_id, left_motor_position, speed);
    this->controller->seekPosition(right_motor_id, right_motor_position, speed);

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

        if (!valid) {
            RCLCPP_FATAL()
            throw std::runtime_error("Xacro configuration not valid for ProjectPerryController; see ROS log");
        }
    }
} // namespace