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

#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

constexpr boost::log::trivial::severity_level LOG_LEVEL = boost::log::trivial::debug;

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

        cfg.device = info_.hardware_parameters["device"];
        cfg.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);

        for (const hardware_interface::ComponentInfo& joint : info_.joints) {
            // DiffBotSystem has exactly two states and one command interface on each joint
            if (joint.command_interfaces.size() != 1) {
                RCLCPP_FATAL(
                        rclcpp::get_logger("RoboticArmControlSystem"),
                        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                        joint.command_interfaces.size()
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
                RCLCPP_FATAL(
                        rclcpp::get_logger("RoboticArmControlSystem"),
                        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2) {
                RCLCPP_FATAL(
                        rclcpp::get_logger("RoboticArmControlSystem"),
                        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                        joint.state_interfaces.size()
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(
                        rclcpp::get_logger("RoboticArmControlSystem"),
                        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
                RCLCPP_FATAL(
                        rclcpp::get_logger("RoboticArmControlSystem"),
                        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY
                );
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        for (const hardware_interface::ComponentInfo& gpio : info_.gpios) {
            // Gripper has exactly one states and one command interface
            if (gpio.command_interfaces.size() != 1) {
                RCLCPP_FATAL(
                        rclcpp::get_logger("RoboticArmControlSystem"),
                        "GPIO '%s' has %zu command interfaces found. 1 expected.", gpio.name.c_str(),
                        gpio.command_interfaces.size()
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (gpio.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(
                        rclcpp::get_logger("RoboticArmControlSystem"),
                        "GPIO '%s' have %s command interfaces found. '%s' expected.", gpio.name.c_str(),
                        gpio.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (gpio.state_interfaces.size() != 1) {
                RCLCPP_FATAL(
                        rclcpp::get_logger("RoboticArmControlSystem"),
                        "GPIO '%s' has %zu state interface. 2 expected.", gpio.name.c_str(),
                        gpio.state_interfaces.size()
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (gpio.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(
                        rclcpp::get_logger("RoboticArmControlSystem"),
                        "GPIO '%s' have '%s' as first state interface. '%s' expected.", gpio.name.c_str(),
                        gpio.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION
                );
                return hardware_interface::CallbackReturn::ERROR;
            }
        }


        // Set the library's log level
        // TODO: Might be nice to use a ROS parameter for this (or somehow tie it to the ROS log level)
        boost::log::core::get()->set_filter(boost::log::trivial::severity >= LOG_LEVEL);

        // Select the StepperAdapter implementation we want to use
        // (For now the only implementation is ArduinoStepperAdapter)
        steppers = std::make_unique<ArduinoStepperAdapter>(info_.joints.size(), std::chrono::milliseconds(100));

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
        RCLCPP_INFO(rclcpp::get_logger("RoboticArmControlSystem"), "Configuring ...please wait...");

        steppers->connect(cfg.device, cfg.baud_rate);

        RCLCPP_INFO(rclcpp::get_logger("RoboticArmControlSystem"), "Successfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RoboticArmControlSystem::on_cleanup(
            const rclcpp_lifecycle::State& /*previous_state*/
    ) {
        RCLCPP_INFO(rclcpp::get_logger("RoboticArmControlSystem"), "Cleaning up ...please wait...");

        steppers->disconnect();

        RCLCPP_INFO(rclcpp::get_logger("RoboticArmControlSystem"), "Successfully cleaned up!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RoboticArmControlSystem::on_activate(
            const rclcpp_lifecycle::State& /*previous_state*/
    ) {
        RCLCPP_INFO(rclcpp::get_logger("RoboticArmControlSystem"), "Activating ...please wait...");

        RCLCPP_INFO(rclcpp::get_logger("RoboticArmControlSystem"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RoboticArmControlSystem::on_deactivate(
            const rclcpp_lifecycle::State& /*previous_state*/
    ) {
        RCLCPP_INFO(rclcpp::get_logger("RoboticArmControlSystem"), "Deactivating ...please wait...");

        RCLCPP_INFO(rclcpp::get_logger("RoboticArmControlSystem"), "Successfully deactivated!");

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
