#ifndef UMRT_ARM_ROS_FIRMWARE_WHEELCONTROLLER_HPP
#define UMRT_ARM_ROS_FIRMWARE_WHEELCONTROLLER_HPP

#include "umrt-arm-ros-firmware/stepper_adapter.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <boost/bimap.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <ros2_j1939_babbler_msgs/msg/servo_control0.hpp>
#include <realtime_tools/realtime_buffer.hpp>

/**
 * Adapter class utilized for ros2_control hardware interface for
 * command interfaces of the CAN-PPM gateway.
 */
class ServoAdapter {
public:
    /** Initializes an ServoAdapter. */
    ServoAdapter(
            const std::size_t num_joints,
            const std::string& topic_name,
            rclcpp::NodeOptions node_options = rclcpp::NodeOptions()
    );

    ~ServoAdapter();

    /** Send servo commands by publishing a J1939 translation topic */
    void writeValues();

    /** Returns the command interface from a certain joint index */
    double& getCommandRef(std::size_t index);

protected:
    std::shared_ptr<rclcpp::Node> node_;

    //  Vector for servo position commands.
    std::vector<double> commands_;

    //  Message Counter
    uint8_t msg_counter_;

    //  J1939 Servo Control Control Publisher
    std::unique_ptr<realtime_tools::RealtimePublisher<ros2_j1939_babbler_msgs::msg::ServoControl0>> realtime_pub_;

};

#endif //UMRT_ARM_ROS_FIRMWARE_WHEELCONTROLLER_HPP
