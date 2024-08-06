//
// Created by Noah Reeder on 2024-08-05.
//

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "keyboard_operator/KeyboardController.h"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardController>());
    rclcpp::shutdown();
    return 0;
}
