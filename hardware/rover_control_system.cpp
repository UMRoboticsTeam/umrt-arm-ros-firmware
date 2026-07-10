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

#include "umrt-arm-ros-firmware/rover_control_system.hpp"
#include "umrt-arm-ros-firmware/wheel_adapter.hpp"
#include "ros2_j1939_babbler_msgs/msg/rover_speed_control.hpp"

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
#include <string>

constexpr boost::log::trivial::severity_level LOG_LEVEL = boost::log::trivial::debug;

namespace umrt_arm_ros_firmware {

    hardware_interface::CallbackReturn DrivetrainControlSystem::on_init(
            const hardware_interface::HardwareInfo& info
    ) {

        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        //  Initialize parameters
        this->declare_parameter<std::string>("rover_speed_topic", "/umrt_ros_controller/RoverSpeedControl/tx");
        std::string rover_speed_topic = this->get_parameter("my_parameter").as_string();

        //  Initialize message counter 
        msg_counter = 0;

        //  Initialize WheelAdapter, and hardware interface node 
        wheels = std::make_unique<WheelAdapter>(info_.joints.size());
        hw_node_ = std::make_shared<rclcpp::Node>("rover_hw_interface_node");

        //  Should get topic name 
        auto standard_pub = hw_node_->create_publisher<ros2_j1939_babbler_msgs::msg::RoverSpeedControl>(
            rover_speed_topic,
            rclcpp::SystemDefaultsQoS()
        );

        realtime_pub_ = std::make_shared<realtime_tools::RealtimePublisher<ros2_j1939_babbler_msgs::msg::RoverSpeedControl>>(standard_pub);

        return hardware_interface::CallbackReturn::SUCCESS;

    }   //  on_init()

    
    std::vector<hardware_interface::StateInterface> DrivetrainControlSystem::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++) {
            // Broadcast Actual Velocity
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &wheels->getVelocityRef(i)));
                
            // Broadcast Actual Position (Encoder Ticks / Radians)
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &wheels->getPositionRef(i)));
        }
        return state_interfaces;
    }   //  export_state_interfaces()

    std::vector<hardware_interface::CommandInterface> DrivetrainControlSystem::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++) {

            //  Info on the ros2_control joint index, used to determine differential joint indexes.
            //  Could be changed later on to dynamically change based on the names of the joint instead
            //  of by numerical order. The order of the joints are based off of the ros2_control.xacro description
            //  joint orders. 
            RCLCPP_INFO(
                rclcpp::get_logger("DrivetrainControlSystem"), 
                "ros2_control Joint Index [%zu] map to URDF Joint %s",
                i, 
                info_.joints[i].name.c_str()
            );

            command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &wheels->getCommandRef(i)));
        }
        return command_interfaces;
    }   //  export_command_interfaces()

    hardware_interface::CallbackReturn DrivetrainControlSystem::on_configure(
            const rclcpp_lifecycle::State& previous_state
    ) {
        RCLCPP_INFO(rclcpp::get_logger("DrivetrainControlSystem"), "Configuring ...please wait...");

        // DO SOME CONFIG HERE 
        // eg. steppers->connect(cfg.device, cfg.baud_rate);

        RCLCPP_INFO(rclcpp::get_logger("DrivetrainControlSystem"), "Successfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }   //  on_configure()

    hardware_interface::CallbackReturn DrivetrainControlSystem::on_cleanup(
            const rclcpp_lifecycle::State& previous_state
    ) {
        RCLCPP_INFO(rclcpp::get_logger("DrivetrainControlSystem"), "Cleaning up ...please wait...");

        //  DO SOME CLEANUP HERE
        //  eg. steppers->disconnect();

        RCLCPP_INFO(rclcpp::get_logger("DrivetrainControlSystem"), "Successfully cleaned up!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }   //  on_cleanup()

    hardware_interface::CallbackReturn DrivetrainControlSystem::on_activate(
            const rclcpp_lifecycle::State& previous_state
    ) {
        RCLCPP_INFO(rclcpp::get_logger("DrivetrainControlSystem"), "Activating ...please wait...");

        RCLCPP_INFO(rclcpp::get_logger("DrivetrainControlSystem"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }   //  on_activate()

    hardware_interface::CallbackReturn DrivetrainControlSystem::on_deactivate(
            const rclcpp_lifecycle::State& previous_state
    ) {
        RCLCPP_INFO(rclcpp::get_logger("DrivetrainControlSystem"), "Deactivating ...please wait...");

        RCLCPP_INFO(rclcpp::get_logger("DrivetrainControlSystem"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }   //  on_deactivate()

    hardware_interface::return_type DrivetrainControlSystem::read(
            const rclcpp::Time& time, const rclcpp::Duration& period
    ) {
        wheels->readValues();
        return hardware_interface::return_type::OK;
    }   //  read()

    hardware_interface::return_type DrivetrainControlSystem::write(
            const rclcpp::Time& time, const rclcpp::Duration& period
    ) {

        //  
        if (realtime_pub_ && realtime_pub_->trylock()) {
            auto &msg = realtime_pub_->msg_;

            //  The indexes could be changed such that instead of hardcoding the index, a variable 
            //  can be changed dynamically based on the names of the joints. 
            double front_left = wheels->getCommandRef(0);
            double rear_left = wheels->getCommandRef(1);
            double front_right = wheels->getCommandRef(2);
            double rear_right = wheels->getCommandRef(3);

            //  Average of the left and right velocities
            //  Will automatically convert it to float32
            msg.left_angular_velocity = (front_left + rear_left) / 2.0;
            msg.right_angular_velocity = (front_right + rear_right) / 2.0;
            
            //  Increment and rollback at 250 (0xFA)
            msg.message_counter = msg_counter; 
            msg_counter = static_cast<uint8_t>((msg_counter + 1) % 251); 

            realtime_pub_->unlockAndPublish();
        }

        return hardware_interface::return_type::OK;
    }   //  write()

} // namespace umrt_arm_ros_firmware


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    umrt_arm_ros_firmware::DrivetrainControlSystem, 
    hardware_interface::SystemInterface
)