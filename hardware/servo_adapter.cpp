#include "umrt-arm-ros-firmware/servo_adapter.hpp"

#include <boost/bimap.hpp>
#include <cmath>


ServoAdapter::ServoAdapter(
        const std::vector<ServoConfig>& servo_configs, const std::string& topic_name, rclcpp::NodeOptions node_options
)
    : node_{ std::make_shared<rclcpp::Node>("servo_adapter", std::move(node_options)) }, commands_{ servo_configs.size() },
      realtime_publishers_(servo_configs.size()) {

    for (std::size_t i = 0 ; i < servo_configs.size(); ++i) {
        auto& servo_config = servo_configs[i];
        auto& realtime_publisher = realtime_publishers_.span[i];

        auto standard_pub = node_->create_publisher<ros2_j1939_babbler_msgs::msg::ServoControl0>(
                topic_name, rclcpp::SystemDefaultsQoS()
        );

        realtime_publisher =
                std::make_unique<realtime_tools::RealtimePublisher<ros2_j1939_babbler_msgs::msg::ServoControl0>>(
                        std::move(standard_pub)
                );
        realtime_publisher->msg_.index = servo_config.id;
        realtime_publisher->msg_.set_angle = servo_config.initial_pos;
        realtime_publisher->msg_.message_counter = 0x00;
        realtime_publisher->msg_.reserved = 0xFFFFFF;
        realtime_publisher->msg_.crc = 0xFF;
    }
}

ServoAdapter::~ServoAdapter() = default;

void ServoAdapter::writeValues() {
    for (std::size_t i = 0; i < commands_.span.size(); ++i) {
        auto& realtime_publisher = realtime_publishers_.span[i];

        // If lock is free send a message, otherwise skip this iteration
        // Up to the controller to determine how it wants to handle message starvation
        if (realtime_publisher && realtime_publisher->trylock()) {
            auto& msg = realtime_publisher->msg_;

            msg.set_angle = static_cast<float>(commands_.span[i]);

            //  Increment and rollback after 250 (0xFA)
            msg.message_counter = static_cast<uint8_t>((msg.message_counter + 1) % 0xFB);

            realtime_publisher->unlockAndPublish();
        }
    }
}

double& ServoAdapter::getCommandRef(const std::size_t index) {
    return this->commands_[index];
}
