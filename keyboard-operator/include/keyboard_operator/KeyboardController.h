//
// Created by Noah Reeder on 2024-08-05.
//

#ifndef KEYBOARD_OPERATOR_KEYBOARDCONTROLLER_H
#define KEYBOARD_OPERATOR_KEYBOARDCONTROLLER_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class KeyboardController : public rclcpp::Node {
public:
    KeyboardController();

protected:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_publisher;

    rclcpp::TimerBase::SharedPtr timer;

    void update();
};


#endif //KEYBOARD_OPERATOR_KEYBOARDCONTROLLER_H
