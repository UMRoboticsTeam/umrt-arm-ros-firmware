//
// Created by Noah on 2024-08-05.
//

#include <string>
#include <chrono>

#include "KeyboardController.h"

constexpr char VEL_COMMAND_TOPIC[] = "/diffbot_base_controller/commands";
constexpr char GRIPPER_COMMAND_TOPIC[] = "/gripper_controller/commands";

constexpr size_t PUBLISHER_QUEUE_DEPTH = 10;

constexpr std::chrono::duration LOOP_RATE = std::chrono::milliseconds(50);

KeyboardController::KeyboardController() : Node("keyboard_operator") {
    this->vel_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(VEL_COMMAND_TOPIC, PUBLISHER_QUEUE_DEPTH);

    timer = this->create_wall_timer(LOOP_RATE, [this]() -> void { this->update(); });
}

void KeyboardController::update() {

}