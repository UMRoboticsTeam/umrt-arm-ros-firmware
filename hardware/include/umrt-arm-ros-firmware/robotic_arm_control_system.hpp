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
    /**
     * ros2_control hardware_interface for the robotic arm.
     * Allows for one of several different controller backends to be selected.
     *
     * Each joint must specify a velocity interface; and if `position_commandable` is enabled, then all must also specify a
     * position command interface. Each joint must also specify a position and velocity state interface, regardless of
     * `position_commandable` value.
     *
     * Requires the following parameters in the xacro:
     *
     * Hardware parameters
     * - `device`: The serial, network, etc. device to communicate to the controller through
     * - `baud_rate`: The baud rate to open the connection with
     * - `controller_type`: The selected controller backend, one of RoboticArmControlSystem::Config::ControllerType
     * - `default_speed`: The speed to move at if a position is commanded without a speed or if speed is 0
     * - `position_commandable`: Whether to expose, and require, position command interfaces for every joint; defaults to false
     * - `gripper_id`: The ID associated with the command interface of the gripper CAN controller; defaults to 0
     * Joint parameters:
     * - `motor_id`: The ID associated with the motor driver attached to the joint
     * - `reduction_factor`: The mechanical reduction factor between the motor and the joint; defaults to 1
     * - `encoder_id`: (Optional) The ID associated with the rotary encoder attached to the joint
     */
    class RoboticArmControlSystem : public hardware_interface::SystemInterface {
    public:
        struct Config {
            enum class ControllerType {
                INVALID,
                ARDUINO,
                MKS
            };

            std::string device;
            int baud_rate;
            bool position_commandable;
            ControllerType controller_type;
            double default_speed;
            std::vector<StepperAdapter::JointInfo> joint_infos{};
            uint16_t gripper_id;

            Config(std::string  device, int baudRate, bool positionCommandable, ControllerType controllerType,
                   double defaultSpeed, uint16_t gripperId);
        };

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
        std::unique_ptr<Config> cfg;
        rclcpp::Logger logger = rclcpp::get_logger("RoboticArmControlSystem");
    };

} // namespace umrt_arm_ros_firmware

#endif // UMRT_ARM_ROS_FIRMWARE_ROBOTIC_ARM_CONTROL_SYSTEM_HPP
