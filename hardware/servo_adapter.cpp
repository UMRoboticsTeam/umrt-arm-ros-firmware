#include "umrt-arm-ros-firmware/servo_adapter.hpp"

#include <boost/bimap.hpp>
#include <cmath>


ServoAdapter::ServoAdapter(const std::size_t num_joints, const std::string& topic_name) {
    this->commands.resize(num_joints);

    msg_counter_ = 0;

    auto standard_pub = hw_node_->create_publisher<ros2_j1939_babbler_msgs::msg::RoverSpeedControl>(
            topic_name,
            rclcpp::SystemDefaultsQoS()
    );

    realtime_pub_ = std::make_unique<realtime_tools::RealtimePublisher<ros2_j1939_babbler_msgs::msg::RoverSpeedControl>>(standard_pub);
}

StepperAdapter::~StepperAdapter() = default;

void StepperAdapter::writeValues() {

    if (realtime_pub_ && realtime_pub_->trylock()) { // TODO: Complete
//        auto &msg = realtime_pub_->msg_;
//
//        //  The indexes could be changed such that instead of hardcoding the index, a variable
//        //  can be changed dynamically based on the names of the joints.
//
//        //  Average of the left and right velocities
//        //  Will automatically convert it to float32
//        msg.left_angular_velocity = (front_left + rear_left) / 2.0;
//        msg.right_angular_velocity = (front_right + rear_right) / 2.0;
//
//        //  Increment and rollback at 250 (0xFA)
//        msg.message_counter = msg_counter_;
//        msg_counter_ = static_cast<uint8_t>((msg_counter_ + 1) % 251);
//
//        realtime_pub_->unlockAndPublish();
    }
    return;
}

double& WheelAdapter::getCommandRef(const std::size_t index) {
    return this->commands[index];
}
