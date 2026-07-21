#include "umrt-arm-ros-firmware/wheel_adapter.hpp"
#include <boost/bimap.hpp>
#include <cmath>


namespace {
    inline double rads_to_rpm(double speed_rads) {
        // const double rads_to_rpm = 30.0 / M_PI;
        //  60 s / (2 * pi) = 30 / pi
        return speed_rads * (30.0 / M_PI);
    }
}

WheelAdapter::WheelAdapter(const std::size_t num_joints, const std::string& topic_name) {

    this->positions.resize(num_joints);
    this->velocities.resize(num_joints);
    this->commands.resize(num_joints);

    msg_counter_ = 0;

    // Create the Node and Publishers
    hw_node_ = std::make_shared<rclcpp::Node>("rover_hw_interface_node");
    
    auto standard_pub = hw_node_->create_publisher<ros2_j1939_babbler_msgs::msg::RoverSpeedControl>(
        topic_name,
        rclcpp::SystemDefaultsQoS()
    );

    realtime_pub_ = std::make_unique<realtime_tools::RealtimePublisher<ros2_j1939_babbler_msgs::msg::RoverSpeedControl>>(standard_pub);

}

WheelAdapter::~WheelAdapter() {
}

void WheelAdapter::connect(const std::string device, const int baud_rate) {
    return;
}

void WheelAdapter::disconnect() {
    return;
}

void WheelAdapter::writeValues() {

    if (realtime_pub_ && realtime_pub_->trylock()) {
        auto &msg = realtime_pub_->msg_;

        //  The indexes could be changed such that instead of hardcoding the index, a variable 
        //  can be changed dynamically based on the names of the joints. 
        double front_left = rads_to_rpm(this->commands[0]);
        double rear_left = rads_to_rpm(this->commands[1]);
        double front_right = rads_to_rpm(this->commands[2]);
        double rear_right = rads_to_rpm(this->commands[3]);

        //  Average of the left and right velocities
        //  Will automatically convert it to float32
        msg.left_angular_velocity = (front_left + rear_left) / 2.0;
        msg.right_angular_velocity = (front_right + rear_right) / 2.0;
        
        //  Increment and rollback at 250 (0xFA)
        msg.message_counter = msg_counter_; 
        msg_counter_ = static_cast<uint8_t>((msg_counter_ + 1) % 251); 

        realtime_pub_->unlockAndPublish();
    }
    return;
}

void WheelAdapter::readValues() {
    return;
}

double& WheelAdapter::getPositionRef(const size_t index) {
    return this->positions[index];
}

double& WheelAdapter::getCommandRef(const std::size_t index) {
    return this->commands[index];
}

double& WheelAdapter::getVelocityRef(const std::size_t index) {
    return this->velocities[index];
}
