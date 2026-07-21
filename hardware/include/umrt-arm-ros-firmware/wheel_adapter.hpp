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
#include <ros2_j1939_babbler_msgs/msg/rover_speed_control.hpp>

/**
 * Adapter class utilized for ros2_control hardware interface for 
 * command interface and state interfaces.
 */
class WheelAdapter {
public:
    /**
    * Initializes an WheelAdapter.
    * The number of joints is inferred from the number of motor IDs provided.
    */
    WheelAdapter(
        const std::size_t num_joints,
        const std::string& topic_name
    );

    ~WheelAdapter();

    /** Does nothing. */
    void connect();

    /** Does nothing. */
    void disconnect();

    /** Send wheel commands by publishing rover wheel speeds to a ROS 2 Topic */
    void writeValues();

    /** Does nothing. */
    void readValues();

    /** Returns the state velocity from a certain joint index */
    double& getVelocityRef(std::size_t index);

    /** Returns the command interface from a certain joint index */
    double& getCommandRef(std::size_t index);

    /** Returns the state position from a certain joint index */
    double& getPositionRef(std::size_t index);

protected:
    
    //  Vector for velocity joint commands.
    std::vector<double> commands;

    //  Vector for joint positions.
    std::vector<double> positions;

    //  Vector for joint velocities.
    std::vector<double> velocities;

    //  Message Counter
    uint8_t msg_counter_;

    //  Hardware Interface ROS 2 Node 
    rclcpp::Node::SharedPtr hw_node_;

    //  J1939 ROS 2 Publisher
    std::unique_ptr<realtime_tools::RealtimePublisher<ros2_j1939_babbler_msgs::msg::RoverSpeedControl>> realtime_pub_;

};

#endif //UMRT_ARM_ROS_FIRMWARE_WHEELCONTROLLER_HPP
