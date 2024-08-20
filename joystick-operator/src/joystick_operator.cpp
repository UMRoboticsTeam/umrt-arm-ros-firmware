//
// Created by Noah on 2024-08-18.
//

#include "rclcpp/rclcpp.hpp"
#include "joystick_operator/joystick_teleop_node.hpp"

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickTeleopNode>());
    rclcpp::shutdown();
    return 0;
}