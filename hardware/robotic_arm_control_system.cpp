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
#include "umrt-arm-ros-firmware/mks_stepper_adapter.hpp"

#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/lexical_cast.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
#include <boost/tokenizer.hpp>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

rclcpp::Logger::Level parse_log_level(const std::string& level);
boost::log::trivial::severity_level ros_log_level_to_boost(const rclcpp::Logger::Level level);

namespace umrt_arm_ros_firmware {
    hardware_interface::CallbackReturn RoboticArmControlSystem::on_init(
            const hardware_interface::HardwareInfo& info
    ) {
        if (
                SystemInterface::on_init(info) !=
                hardware_interface::CallbackReturn::SUCCESS
        ) {
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

        cfg.device = info_.hardware_parameters["device"];
        cfg.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
        cfg.controller_type = Config::controller_type_from_string(info_.hardware_parameters["controller_type"]);

        for (const hardware_interface::ComponentInfo& joint : info_.joints) {
            // DiffBotSystem has exactly two states and one command interface on each joint
            if (joint.command_interfaces.size() != 1) {
                RCLCPP_FATAL(
                        this->logger,
                        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                        joint.command_interfaces.size()
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
                RCLCPP_FATAL(
                        this->logger,
                        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2) {
                RCLCPP_FATAL(
                        this->logger,
                        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                        joint.state_interfaces.size()
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(
                        this->logger,
                        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
                RCLCPP_FATAL(
                        this->logger,
                        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            uint16_t motor_id;
            if (const auto x = info_.hardware_parameters.find("motor_id"); x == info_.hardware_parameters.end()) {
                RCLCPP_FATAL(
                        this->logger,
                        "Joint '%s' does not have a motor ID specified", joint.name.c_str()
                );
                return hardware_interface::CallbackReturn::ERROR;
            } else {
                motor_id = std::stoi(x->second);
            }

            uint16_t reduction_factor;
            if (const auto x = info_.hardware_parameters.find("reduction_factor"); x == info_.hardware_parameters.end()) {
                reduction_factor = 1;
            } else {
                reduction_factor = std::stoi(x->second);
            }

            cfg.joint_infos.emplace_back(motor_id, reduction_factor);
        }

        for (const hardware_interface::ComponentInfo& gpio : info_.gpios) {
            // Gripper has exactly one states and one command interface
            if (gpio.command_interfaces.size() != 1) {
                RCLCPP_FATAL(
                        this->logger,
                        "GPIO '%s' has %zu command interfaces found. 1 expected.", gpio.name.c_str(),
                        gpio.command_interfaces.size()
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (gpio.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(
                        this->logger,
                        "GPIO '%s' have %s command interfaces found. '%s' expected.", gpio.name.c_str(),
                        gpio.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (gpio.state_interfaces.size() != 1) {
                RCLCPP_FATAL(
                        this->logger,
                        "GPIO '%s' has %zu state interface. 2 expected.", gpio.name.c_str(),
                        gpio.state_interfaces.size()
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (gpio.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(
                        this->logger,
                        "GPIO '%s' have '%s' as first state interface. '%s' expected.", gpio.name.c_str(),
                        gpio.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION
                );
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        // Select the StepperAdapter implementation we want to use
        switch (cfg.controller_type) {
            case Config::ControllerType::ARDUINO:
                steppers = std::make_unique<ArduinoStepperAdapter>(info_.joints.size(), std::chrono::milliseconds(100));
                break;
            case Config::ControllerType::MKS:
                steppers = std::make_unique<MksStepperAdapter>(cfg.device, cfg.joint_infos, std::chrono::milliseconds(100));
                break;
            default: throw std::invalid_argument("Unknown controller type");
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> RoboticArmControlSystem::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++) {
            state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &steppers->getPositionRef(i));
            state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &steppers->getVelocityRef(i));
        }

        state_interfaces.emplace_back(info_.gpios[0].name, hardware_interface::HW_IF_POSITION, &steppers->getGripperPositionRef());

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> RoboticArmControlSystem::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++) {
            command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &steppers->getCommandRef(i));
        }

        command_interfaces.emplace_back(info_.gpios[0].name, hardware_interface::HW_IF_POSITION, &steppers->getGripperPositionCommandRef());

        return command_interfaces;
    }

    hardware_interface::CallbackReturn RoboticArmControlSystem::on_configure(
            const rclcpp_lifecycle::State& /*previous_state*/
    ) {
        RCLCPP_INFO(this->logger, "Configuring ...please wait...");

        steppers->connect(cfg.device, cfg.baud_rate);

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
        steppers->readValues();

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RoboticArmControlSystem::write(
            const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/
    ) {
        steppers->setValues();

        return hardware_interface::return_type::OK;
    }

    // What about onShutdown???

} // namespace umrt_arm_ros_firmware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
        umrt_arm_ros_firmware::RoboticArmControlSystem, hardware_interface::SystemInterface
)

/**
 * Converts a log level string to the associated constant.
 * One would think this would be part of rclutils, but I couldn't find it.
 *
 * @throws std::runtime_error if an invalid log level is provided
 * @param level level string
 * @return level constant
 */
rclcpp::Logger::Level parse_log_level(const std::string& level) {
    if (level == "Debug") { return rclcpp::Logger::Level::Debug; }
    if (level == "Info") { return rclcpp::Logger::Level::Info; }
    if (level == "Warn") { return rclcpp::Logger::Level::Warn; }
    if (level == "Error") { return rclcpp::Logger::Level::Error; }
    if (level == "Fatal") { return rclcpp::Logger::Level::Fatal; }
    if (level == "Unset") { return rclcpp::Logger::Level::Unset; }
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