// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "umrt-arm-ros-firmware/robotic_arm_control_system.hpp"
#include "umrt-arm-ros-firmware/arduino_stepper_adapter.hpp"
#include "umrt-arm-ros-firmware/project_perry_controller.hpp"

#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>

#include "enchantum_single_header.hpp"

#include <chrono>
#include <optional>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace {
    using Config = umrt_arm_ros_firmware::RoboticArmControlSystem::Config;
    rclcpp::Logger::Level parse_log_level(const std::string& level);
    boost::log::trivial::severity_level ros_log_level_to_boost(const rclcpp::Logger::Level level);
    std::optional<Config> parse_controller_config(const std::unordered_map<std::string, std::string>& hardware_parameters, rclcpp::Logger& logger
    );
    hardware_interface::CallbackReturn
    validate_joint(const hardware_interface::ComponentInfo& joint, rclcpp::Logger& logger, bool position_commandable);
    hardware_interface::CallbackReturn validate_gpio(const hardware_interface::ComponentInfo& gpio, rclcpp::Logger& logger);
}

namespace umrt_arm_ros_firmware {
    RoboticArmControlSystem::Config::Config(std::string device, int baudRate, bool positionCommandable, ControllerType controllerType,
           double defaultSpeed, uint16_t gripperId)
                : device(std::move(device)), baud_rate(baudRate), position_commandable(positionCommandable),
                  controller_type(controllerType), default_speed(defaultSpeed),
                  gripper_id(gripperId) {}

    hardware_interface::CallbackReturn RoboticArmControlSystem::on_init(const hardware_interface::HardwareInfo& info) {
        if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Set the log level to specified, or DEBUG if not
        rclcpp::Logger::Level log_level;
        if (const auto x = info_.hardware_parameters.find("log_level"); x == info_.hardware_parameters.end()) {
            log_level = rclcpp::Logger::Level::Debug;
        } else {
            log_level = parse_log_level(x->second);
        }
        this->logger.set_level(log_level);
        boost::log::core::get()->set_filter(boost::log::trivial::severity >= ros_log_level_to_boost(log_level));

        // Parse the top-level controller config parameters, and make sure they're valid
        if (std::optional<Config> x = parse_controller_config(info_.hardware_parameters, this->logger); x.has_value()) { cfg = std::make_unique<Config>(std::move(*x));}
        else { return hardware_interface::CallbackReturn::ERROR; }

        for (const hardware_interface::ComponentInfo& joint : info_.joints) {
            if (auto x = validate_joint(joint, this->logger, cfg->position_commandable); x != hardware_interface::CallbackReturn::SUCCESS) { return x; }

            uint16_t motor_id;
            if (const auto x = joint.parameters.find("motor_id"); x == joint.parameters.end()) {
                RCLCPP_FATAL(
                        this->logger,
                        "Joint '%s' does not have a motor ID specified", joint.name.c_str()
                );
                return hardware_interface::CallbackReturn::ERROR;
            } else {
                motor_id = std::stoi(x->second);
            }

            uint32_t encoder_id;
            if (const auto x = joint.parameters.find("encoder_id"); x == joint.parameters.end()) {
                encoder_id = 0;
            } else {
                encoder_id = std::stoi(x->second);
            }

            uint16_t reduction_factor;
            if (const auto x = joint.parameters.find("reduction_factor"); x == joint.parameters.end()) {
                reduction_factor = 1;
            } else {
                reduction_factor = std::stoi(x->second);
            }

            bool differential;
            if (const auto x = joint.parameters.find("differential"); x == joint.parameters.end()) {
                differential = false;
            } else {
                differential = x->second == "true";
            }

            cfg->joint_infos.emplace_back(motor_id, encoder_id, reduction_factor, differential);
        }

        for (const hardware_interface::ComponentInfo& gpio : info_.gpios) {
            if (auto x = validate_gpio(gpio, this->logger); x != hardware_interface::CallbackReturn::SUCCESS) { return x; }
        }

        // Select the StepperAdapter implementation we want to use
        RCLCPP_INFO_STREAM(this->logger, "Creating controller type " << enchantum::to_string(cfg->controller_type) << ", with position_commandable set to " << (cfg->position_commandable ? "true" : "false"));
        try {
            switch (cfg->controller_type) {
                case Config::ControllerType::ARDUINO:
                    if (cfg->position_commandable) { throw std::invalid_argument("ArduinoStepperAdapter cannot be used with position_commandable"); }
                    steppers = std::make_unique<ArduinoStepperAdapter>(info_.joints.size(), std::chrono::milliseconds(100));
                    break;
                case Config::ControllerType::MKS:
                    if (!cfg->position_commandable) { throw std::invalid_argument("ProjectPerryController must be used with position_commandable"); }
                    steppers = std::make_unique<ProjectPerryController>(cfg->device, cfg->joint_infos, cfg->gripper_id, cfg->default_speed, std::chrono::milliseconds(100), this->logger);
                    break;
                default: throw std::invalid_argument("Unknown controller type");
            }
        } catch (std::exception& e) { // If this fails for any reason, intercept to dump our XML and then continue
            RCLCPP_FATAL_STREAM(this->logger, "Failed to create controller; XML parameters:\n" << info_.original_xml.c_str());
            throw e;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> RoboticArmControlSystem::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++) {
            state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &steppers->getStateVelocityRef(i));
            state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &steppers->getStatePositionRef(i));
        }

        state_interfaces.emplace_back(info_.gpios[0].name, hardware_interface::HW_IF_POSITION, &steppers->getGripperPositionRef());
        state_interfaces.emplace_back(info_.gpios[0].name, hardware_interface::HW_IF_VELOCITY, &steppers->getGripperVelocityRef());

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> RoboticArmControlSystem::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++) {
            command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &steppers->getCommandVelocityRef(i));
            if (cfg->position_commandable) { command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &steppers->getCommandPositionRef(i)); }
        }

        command_interfaces.emplace_back(info_.gpios[0].name, hardware_interface::HW_IF_POSITION, &steppers->getGripperPositionCommandRef());

        return command_interfaces;
    }

    hardware_interface::CallbackReturn RoboticArmControlSystem::on_configure(
            const rclcpp_lifecycle::State& /*previous_state*/
    ) {
        RCLCPP_INFO(this->logger, "Configuring ...please wait...");

        steppers->connect(cfg->device, cfg->baud_rate);

        RCLCPP_INFO(this->logger, "Successfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RoboticArmControlSystem::on_cleanup(
            const rclcpp_lifecycle::State& /*previous_state*/
    ) {
        RCLCPP_INFO(this->logger, "Cleaning up ...please wait...");

        steppers->disconnect();

        RCLCPP_INFO(this->logger, "Successfully cleaned up!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RoboticArmControlSystem::on_activate(
            const rclcpp_lifecycle::State& /*previous_state*/
    ) {
        RCLCPP_INFO(this->logger, "Activating ...please wait...");

        RCLCPP_INFO(this->logger, "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RoboticArmControlSystem::on_deactivate(
            const rclcpp_lifecycle::State& /*previous_state*/
    ) {
        RCLCPP_INFO(this->logger, "Deactivating ...please wait...");

        RCLCPP_INFO(this->logger, "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type RoboticArmControlSystem::read(
            const rclcpp::Time& /*time*/, const rclcpp::Duration& period
    ) {
        (void)period; // We currently don't care about the time between read calls, so stop warning about it being unused
        steppers->readValues();

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RoboticArmControlSystem::write(
            const rclcpp::Time& /*time*/, const rclcpp::Duration& period
    ) {
        (void)period; // We currently don't care about the time between write calls, so stop warning about it being unused

        steppers->setValues();

        return hardware_interface::return_type::OK;
    }

    // What about onShutdown???

} // namespace umrt_arm_ros_firmware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
        umrt_arm_ros_firmware::RoboticArmControlSystem, hardware_interface::SystemInterface
)

namespace {
    /**
     * Converts a log level string to the associated constant.
     * One would think this would be part of rclutils, but I couldn't find it.
     *
     * @throws std::runtime_error if an invalid log level is provided
     * @param level level string
     * @return level constant
     */
    rclcpp::Logger::Level parse_log_level(const std::string& level) {
        if (level == "debug") { return rclcpp::Logger::Level::Debug; }
        if (level == "info") { return rclcpp::Logger::Level::Info; }
        if (level == "warn") { return rclcpp::Logger::Level::Warn; }
        if (level == "error") { return rclcpp::Logger::Level::Error; }
        if (level == "fatal") { return rclcpp::Logger::Level::Fatal; }
        if (level == "unset") { return rclcpp::Logger::Level::Unset; }
        throw std::runtime_error("Invalid log level");
    }

    /**
     * Converts the provided ROS2 log level to the corresponding Boost log level.
     * Since ROS2 does not support have a log level equivalent to `trace`, it is not available here.
     *
     * @throws std::runtime_error if an invalid log level or Unset is provided
     * @param level ROS2 log level
     * @return Boost log level
     */
    boost::log::trivial::severity_level ros_log_level_to_boost(const rclcpp::Logger::Level level) {
        switch (level) {
            case rclcpp::Logger::Level::Debug: return boost::log::trivial::severity_level::debug;
            case rclcpp::Logger::Level::Info: return boost::log::trivial::severity_level::info;
            case rclcpp::Logger::Level::Warn: return boost::log::trivial::severity_level::warning;
            case rclcpp::Logger::Level::Error: return boost::log::trivial::severity_level::error;
            case rclcpp::Logger::Level::Fatal: return boost::log::trivial::severity_level::fatal;
            case rclcpp::Logger::Level::Unset:
            default: throw std::runtime_error("Invalid log level");
        }
    }

    /**
     * Helper function which validates that the top-level controller config parameters provided in the xacro are plausible,
     * and parses them into a Config structure.
     * @param hardware_parameters the std::unordered_map<std::string, std::string> holding provided hardware config parameters
     * @return std::optional<umrt_arm_ros_firmware::RoboticArmControlSystem::Config> containing the constructed Config structure,
     * if not present an unrecoverable error occurred during parsing
     */
    std::optional<Config> parse_controller_config(const std::unordered_map<std::string, std::string>& hardware_parameters, rclcpp::Logger& logger
    ) {
        // Check device string is present
        std::string device;
        if (auto x = hardware_parameters.find("device"); x == hardware_parameters.end()) {
            RCLCPP_FATAL(logger, "Parameter 'device' not specified");
            return std::nullopt;
        }
        else {
            device = x->second;
        }

        // Check baud rate is present and an integer
        int baud_rate;
        if (auto x = hardware_parameters.find("baud_rate"); x == hardware_parameters.end()) {
            RCLCPP_FATAL(logger, "Parameter 'baud_rate' not specified");
           return std::nullopt;
        }
        else {
            try {
                baud_rate = std::stoi(x->second);
            }
            catch(const std::invalid_argument& e) {
                RCLCPP_FATAL(
                        logger, "Parameter 'baud_rate' is not integral: %s",
                        x->second.c_str()
                );
               return std::nullopt;
            }
            catch (const std::out_of_range& e) {
                RCLCPP_FATAL(
                        logger, "Parameter 'baud_rate' has invalid value: %s",
                        x->second.c_str()
                );
               return std::nullopt;
            }
        }

        // Check position_commandable is present and either "true" or "false"
        bool position_commandable;
        if (auto x = hardware_parameters.find("position_commandable"); x == hardware_parameters.end()) {
            RCLCPP_FATAL(logger, "Parameter 'position_commandable' not specified");
           return std::nullopt;
        }
        else if (x->second != "true" && x->second != "false"){
            RCLCPP_FATAL(logger, "Parameter 'position_commandable' must be either 'true' or 'false', got: %s", x->second.c_str());
           return std::nullopt;
        }
        else {
            position_commandable = x->second == "true";
        }

        // Extract default speed, and assign to default value if not present
        double default_speed;
        if (auto x = hardware_parameters.find("default_speed"); x == hardware_parameters.end()) { default_speed = 0; }
        else if (x->second.empty()) { default_speed = 0; }
        else {
            try {
                default_speed = std::stod(x->second);
            }
            catch(const std::invalid_argument& e) {
                RCLCPP_FATAL(
                        logger, "Parameter 'default_speed' is not numerical: %s",
                        x->second.c_str()
                );
               return std::nullopt;
            }
            catch (const std::out_of_range& e) {
                RCLCPP_FATAL(
                        logger, "Parameter 'default_speed' has invalid value: %s",
                        x->second.c_str()
                );
            }
        }

        // Check controller type is one of the options
        Config::ControllerType controller_type;
        if (auto x = hardware_parameters.find("controller_type"); x == hardware_parameters.end()) {
            RCLCPP_FATAL(logger, "Parameter 'controller_type' not specified");
           return std::nullopt;
        }
        else if (!enchantum::cast<Config::ControllerType>(x->second).has_value()) {
            RCLCPP_FATAL(
                    logger, "Parameter 'controller_type' has invalid value: %s",
                    hardware_parameters.at("controller_type").c_str()
            );
           return std::nullopt;
        }
        else {
            controller_type = *enchantum::cast<Config::ControllerType>(x->second);
        }

        // Extract gripper ID, and assign default value if not present
        uint16_t gripper_id;
        if (auto x = hardware_parameters.find("gripper_id"); x == hardware_parameters.end()) { gripper_id = 0; }
        else if (x->second.empty()) { gripper_id = 0; }
        else {
            try {
                gripper_id = std::stoi(x->second);
            }
            catch(const std::invalid_argument& e) {
                RCLCPP_FATAL(
                        logger, "Parameter 'gripper_id' is not integral: %s",
                        x->second.c_str()
                );
               return std::nullopt;
            }
            catch(const std::out_of_range& e) {
                RCLCPP_FATAL(
                        logger, "Parameter 'gripper_id' has invalid value: %s",
                        x->second.c_str()
                );
               return std::nullopt;
            }
        }
        // TODO: Make position_commandable a "command mode" enum, would be much more user friendly/clear

        return Config(device, baud_rate, position_commandable, controller_type, default_speed, gripper_id);
    }

    /**
     * Helper function which validates that the joint information provided in the xacro is as expected.
     * @param joint the hardware_interface::ComponentInfo representing the joint to validate
     * @return hardware_interface::CallbackReturn::SUCCESS if requirements met, hardware_interface::CallbackReturn::ERROR if not
     */
    hardware_interface::CallbackReturn
    validate_joint(const hardware_interface::ComponentInfo& joint, rclcpp::Logger& logger, const bool position_commandable) {
        // Two cases here: All RoboticArmControlSystem joints must have a velocity command interface, but if position_commandable
        //      is true then they must all also have a position command interface.

        size_t expected_command_interfaces = 1;
        if (position_commandable) { expected_command_interfaces = 2; }

        if (joint.command_interfaces.size() != expected_command_interfaces) {
            RCLCPP_FATAL(
                    logger, "Expected joint '%s' to have %zu *COMMAND* interface(s), found %zu", joint.name.c_str(),
                    expected_command_interfaces, joint.command_interfaces.size()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        // We don't know which appears first; could be velocity, could be position
        if (joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY) {
            ; // Acceptable
        } else if (joint.command_interfaces[1].name == hardware_interface::HW_IF_VELOCITY) {
            ; // Acceptable
        } else {
            RCLCPP_FATAL(
                    logger, "Expected joint '%s' to have a velocity *COMMAND* interface, did not find one",
                    joint.name.c_str()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (position_commandable) {
            if (joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION) {
                ; // Acceptable
            } else if (joint.command_interfaces[1].name == hardware_interface::HW_IF_POSITION) {
                ; // Acceptable
            } else {
                RCLCPP_FATAL(
                        logger,
                        "Expected joint '%s' to have a position *COMMAND* interface since position_commandable set to true, but did not find one",
                        joint.name.c_str()
                );
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(
                    logger, "Expected joint '%s' to have 2 *STATE* interfaces, found  %zu", joint.name.c_str(),
                    joint.state_interfaces.size()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        // We don't know which appears first; could be velocity, could be position
        if (joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY) {
            ; // Acceptable
        } else if (joint.state_interfaces[1].name == hardware_interface::HW_IF_VELOCITY) {
            ; // Acceptable
        } else {
            RCLCPP_FATAL(
                    logger, "Expected joint '%s' to have a velocity *STATE* interface, did not find one", joint.name.c_str()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION) {
            ; // Acceptable
        } else if (joint.state_interfaces[1].name == hardware_interface::HW_IF_POSITION) {
            ; // Acceptable
        } else {
            RCLCPP_FATAL(
                    logger, "Expected joint '%s' to have a position *STATE* interface, but did not find one",
                    joint.name.c_str()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    /**
     * Helper function which validates that the gpio information provided in the xacro is as expected.
     * @param gpio the hardware_interface::ComponentInfo representing the gpio to validate
     * @return hardware_interface::CallbackReturn::SUCCESS if requirements met, hardware_interface::CallbackReturn::ERROR if not
     */
    hardware_interface::CallbackReturn validate_gpio(const hardware_interface::ComponentInfo& gpio, rclcpp::Logger& logger) {
        // Gripper has exactly one states and one command interface
        if (gpio.command_interfaces.size() != 1) {
            RCLCPP_FATAL(
                    logger, "GPIO '%s' has %zu command interfaces found. 1 expected.", gpio.name.c_str(),
                    gpio.command_interfaces.size()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (gpio.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(
                    logger, "GPIO '%s' have %s command interfaces found. '%s' expected.", gpio.name.c_str(),
                    gpio.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (gpio.state_interfaces.size() != 2) {
            RCLCPP_FATAL(
                    logger, "GPIO '%s' has %zu state interface. 2 expected.", gpio.name.c_str(), gpio.state_interfaces.size()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (gpio.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(
                    logger, "GPIO '%s' have '%s' as first state interface. '%s' expected.", gpio.name.c_str(),
                    gpio.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (gpio.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(
                    logger, "GPIO '%s' have '%s' as second state interface. '%s' expected.", gpio.name.c_str(),
                    gpio.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }
}