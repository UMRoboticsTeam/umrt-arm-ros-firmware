#ifndef UMRT_ARM_ROS_FIRMWARE_ROVER_CONTROL_SYSTEM_HPP
#define UMRT_ARM_ROS_FIRMWARE_ROVER_CONTROL_SYSTEM_HPP

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
#include "umrt-arm-ros-firmware/wheel_adapter.hpp"

namespace umrt_arm_ros_firmware {
    class RoverControlSystem : public hardware_interface::SystemInterface {
        struct Config {
        };


    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(RoverControlSystem);

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
        std::unique_ptr<WheelController> wheels;
        //Config cfg;
    };

} // namespace umrt_arm_ros_firmware
#endif // UMRT_ARM_ROS_FIRMWARE_ROVER_CONTROL_SYSTEM_HPP
