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

#include "umrt-arm-ros-firmware/servo_control_system.hpp"
#include "umrt-arm-ros-firmware/wheel_adapter.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <vector>
#include <string>

namespace umrt_arm_ros_firmware {

    hardware_interface::CallbackReturn ServoControlSystem::on_init(
            const hardware_interface::HardwareInfo& info
    ) {

        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        std::string servo_control_topic = info.hardware_parameters.at("servo_control_topic");

        servos_ = std::make_unique<ServoAdapter>(info.joints.size(), servo_control_topic);

        return hardware_interface::CallbackReturn::SUCCESS;

    }   //  on_init()


    std::vector<hardware_interface::StateInterface> ServoControlSystem::export_state_interfaces() {
        // There are no state interfaces
        return {};
    }   //  export_state_interfaces()

    std::vector<hardware_interface::CommandInterface> ServoControlSystem::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_POSITION, &servos_->getCommandRef(i)));
        }
        return command_interfaces;
    }   //  export_command_interfaces()

    hardware_interface::CallbackReturn ServoControlSystem::on_configure(
            const rclcpp_lifecycle::State& /*previous_state*/
    ) {
        RCLCPP_INFO(logger_, "Configuring...");
        RCLCPP_INFO(logger_, "Successfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }   //  on_configure()

    hardware_interface::CallbackReturn ServoControlSystem::on_cleanup(
            const rclcpp_lifecycle::State& /*previous_state*/
    ) {
        RCLCPP_INFO(logger_, "Cleaning up...");
        RCLCPP_INFO(logger_, "Successfully cleaned up!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }   //  on_cleanup()

    hardware_interface::CallbackReturn ServoControlSystem::on_activate(
            const rclcpp_lifecycle::State& /*previous_state*/
    ) {
        RCLCPP_INFO(logger_, "Activating...");
        RCLCPP_INFO(logger_, "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }   //  on_activate()

    hardware_interface::CallbackReturn ServoControlSystem::on_deactivate(
            const rclcpp_lifecycle::State& /*previous_state*/
    ) {
        RCLCPP_INFO(logger_, "Deactivating...");
        RCLCPP_INFO(logger_, "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }   //  on_deactivate()

    hardware_interface::return_type ServoControlSystem::read(
            const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/
    ) {
        // No state to read
        return hardware_interface::return_type::OK;
    }   //  read()

    hardware_interface::return_type ServoControlSystem::write(
            const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/
    ) {
        servos_->writeValues();
        return hardware_interface::return_type::OK;
    }   //  write()

} // namespace umrt_arm_ros_firmware


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
        umrt_arm_ros_firmware::ServoControlSystem, hardware_interface::SystemInterface
)