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

#ifndef UMRT_ARM_ROS_FIRMWARE_ROBOTIC_ARM_CONTROL_SYSTEM_HPP
#define UMRT_ARM_ROS_FIRMWARE_ROBOTIC_ARM_CONTROL_SYSTEM_HPP

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "umrt-arm-ros-firmware/visibility_control.h"

#include "stepper_adapter.hpp"

namespace umrt_arm_ros_firmware {
    class RoboticArmControlSystem : public hardware_interface::SystemInterface {
        struct Config {
            enum class ControllerType {
                INVALID,
                ARDUINO,
                MKS
            };

            std::string device;
            int baud_rate = 0;
            ControllerType controller_type = ControllerType::INVALID;
            std::vector<uint16_t> motor_ids{};

            static ControllerType controller_type_from_string(const std::string& controller_type) {
                if (controller_type == "ARDUINO") { return ControllerType::ARDUINO; }
                if (controller_type == "MKS") { return ControllerType::MKS; }
                throw std::invalid_argument((std::stringstream() << "Invalid controller type: " << controller_type).str());
            }
        };

    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(RoboticArmControlSystem);

        UMRT_ARM_ROS_FIRMWARE_PUBLIC
        hardware_interface::CallbackReturn on_init(
                const hardware_interface::HardwareInfo& info
        ) override;

        UMRT_ARM_ROS_FIRMWARE_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        UMRT_ARM_ROS_FIRMWARE_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        UMRT_ARM_ROS_FIRMWARE_PUBLIC
        hardware_interface::CallbackReturn on_configure(
                const rclcpp_lifecycle::State& previous_state
        ) override;

        UMRT_ARM_ROS_FIRMWARE_PUBLIC
        hardware_interface::CallbackReturn on_cleanup(
                const rclcpp_lifecycle::State& previous_state
        ) override;

        UMRT_ARM_ROS_FIRMWARE_PUBLIC
        hardware_interface::CallbackReturn on_activate(
                const rclcpp_lifecycle::State& previous_state
        ) override;

        UMRT_ARM_ROS_FIRMWARE_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(
                const rclcpp_lifecycle::State& previous_state
        ) override;

        UMRT_ARM_ROS_FIRMWARE_PUBLIC
        hardware_interface::return_type read(
                const rclcpp::Time& time, const rclcpp::Duration& period
        ) override;

        UMRT_ARM_ROS_FIRMWARE_PUBLIC
        hardware_interface::return_type write(
                const rclcpp::Time& time, const rclcpp::Duration& period
        ) override;

    private:
        std::unique_ptr<StepperAdapter> steppers;
        rclcpp::Logger logger = rclcpp::get_logger("RoboticArmControlSystem");
        Config cfg;
    };

} // namespace umrt_arm_ros_firmware

#endif // UMRT_ARM_ROS_FIRMWARE_ROBOTIC_ARM_CONTROL_SYSTEM_HPP
