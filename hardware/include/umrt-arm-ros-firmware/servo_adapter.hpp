#ifndef UMRT_ARM_ROS_FIRMWARE_WHEELCONTROLLER_HPP
#define UMRT_ARM_ROS_FIRMWARE_WHEELCONTROLLER_HPP

#include "umrt-arm-ros-firmware/stepper_adapter.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>
#include <vector>
#include <string>
#include <span>

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
            const std::vector<std::uint8_t>& servo_ids,
            const std::string& topic_name,
            rclcpp::NodeOptions node_options = rclcpp::NodeOptions()
    );

    ~ServoAdapter();

    /** Send servo commands by publishing a J1939 translation topic */
    void writeValues();

    /** Returns the command interface from a certain joint index */
    double& getCommandRef(std::size_t index);

protected:
    /**
     * Container of fixed size where size is determined at runtime.
     * @tparam T type to contain
     */
    template<typename T>
    struct RuntimeFixedContainer {
        explicit RuntimeFixedContainer<T>(const std::size_t size)
            : backing_storage{ std::make_unique<T[]>(size) }, span{ backing_storage.get(), size } {}

        std::unique_ptr<T[]> backing_storage;
        std::span<T> span;
    };

    std::shared_ptr<rclcpp::Node> node_;     // Node to create publisher under - not spun because we don't need callbacks
    RuntimeFixedContainer<double> commands_; //  Container for servo position commands.
    RuntimeFixedContainer<std::unique_ptr<realtime_tools::RealtimePublisher<ros2_j1939_babbler_msgs::msg::ServoControl0>>>
            realtime_publishers_;
};

#endif //UMRT_ARM_ROS_FIRMWARE_WHEELCONTROLLER_HPP
