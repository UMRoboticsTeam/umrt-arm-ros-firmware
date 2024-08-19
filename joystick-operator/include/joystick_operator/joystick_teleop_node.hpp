//
// Created by Noah on 2024-08-18.
//

#ifndef ARM_FIRMWARE_JOYSTICK_TELEOP_NODE_HPP
#define ARM_FIRMWARE_JOYSTICK_TELEOP_NODE_HPP

#include <string>
#include <unordered_map>
#include <tuple>

#include <boost/variant.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class JoystickTeleopNode : public rclcpp::Node {
public:
    static inline const std::unordered_map<std::string, std::tuple<boost::variant<int, double, std::string>, std::string>> DEFAULT_PARAMETERS = {
        {"deadman_button", {0, "Joystick button to enable movement (int)"}},
        {"slow_button", {1, "Joystick button to move at a slower speed (int)"}},
        {"gripper_open_button", {2, "Joystick button to open the gripper (int)"}},
        {"gripper_close_button", {3, "Joystick button to close the gripper (int)"}},
        {"axis_x", {0, "Joystick axis corresponding the X axis (int)"}},
        {"axis_y", {1, "Joystick axis corresponding to the Y axis (int)"}},
        {"axis_z", {2, "Joystick axis corresponding to the Z axis (int)"}},
        {"axis_speed", {20.0, "Speed to move along an axis when the joystick is fully deflected, in motor RPM (double)"}},
        {"gripper_speed", {50.0, "Speed to move the gripper at when a button is held, in (% of range)/s (double)"}},
        {"slow_modifier", {0.1, "Multiplier to apply to speeds when the slow button is held (double)"}},
        {"gripper_min", {0, "Minimum value to allow gripper to be set to, also used in conjunction with gripper_max to determine the range for gripper_speed (int)"}},
        {"gripper_max", {180, "Maximum value to allow gripper to be set to, also used in conjunction with gripper_min to determine the range for gripper_speed (int)"}},
        {"vel_topic", {"/cmd_vel", "Topic to publish joint speeds to (string)"}},
        {"gripper_topic", {"/gripper_pos", "Topic to publish gripper positions to (string)"}},
        {"joy_topic", {"/joy", "Topic to read Joy messages from (string)"}}
    };
    static constexpr int PUBLISHER_QUEUE_DEPTH = 10;

    JoystickTeleopNode();

protected:
    void initializeParameters();

    void handleJoy(const sensor_msgs::msg::Joy::ConstSharedPtr& msg);

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;

    // Should be treated as const, not const because initializing with initializer list was inconvenient and ugly
    size_t deadman_button;
    size_t slow_button;
    size_t gripper_open_button;
    size_t gripper_close_button;
    size_t axis_x;
    size_t axis_y;
    size_t axis_z;
    double axis_speed;
    double gripper_speed;
    double slow_modifier;
    int gripper_min;
    int gripper_max;
    std::string vel_topic;
    std::string gripper_topic;
    std::string joy_topic;
};


#endif //ARM_FIRMWARE_JOYSTICK_TELEOP_NODE_HPP
