//
// Created by Noah on 2024-08-18.
//

#include "joystick_operator/joystick_teleop_node.hpp"
#include <boost/math/special_functions/sign.hpp>

const std_msgs::msg::Float64MultiArray JoystickTeleopNode::ZERO_VEL = std_msgs::msg::Float64MultiArray().set__data({ 0.0, 0.0, 0.0 });

// Helper functions
double getAxisValue(const sensor_msgs::msg::Joy::ConstSharedPtr& msg, const size_t axis);
int getButtonValue(const sensor_msgs::msg::Joy::ConstSharedPtr& msg, const size_t button);

#define SGN(x)

JoystickTeleopNode::JoystickTeleopNode() : Node("joystick_teleop") {
    this->initializeParameters();

    this->last_gripper.data = { 0.0 };
    this->last_vel = ZERO_VEL;
    this->last_time = std::chrono::steady_clock::now();
    this->gripper_moving = false;

    this->vel_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(this->vel_topic, JoystickTeleopNode::PUBLISHER_QUEUE_DEPTH);
    this->gripper_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(this->gripper_topic, JoystickTeleopNode::PUBLISHER_QUEUE_DEPTH);
    this->joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
            this->joy_topic,
            10,
            [this](const sensor_msgs::msg::Joy::ConstSharedPtr& msg) { this->handleJoy(msg); }
    );

    // Convert gripper speed to (servo units)/s
    this->gripper_speed_converted = (this->gripper_max - this->gripper_min) * this->gripper_speed;

    this->movement_enabled = false;
}

void JoystickTeleopNode::handleJoy(const sensor_msgs::msg::Joy::ConstSharedPtr& msg) {
    // Check that the deadman switch is engaged
    // Since buttons is an array, we need to check that it is longer than the deadman button index first
    if (msg->buttons.size() > this->deadman_button && msg->buttons[this->deadman_button]) {
        // Check if slow-mode enabled
        double multiplier = getButtonValue(msg, this->slow_button) ? this->slow_modifier : 1.0;

        // Construct velocity message
        // This is simple since we directly map joystick value to joint velocity, and joystick value is already normalized
        std_msgs::msg::Float64MultiArray vel;
        std_msgs::msg::Float64MultiArray gripper;
        vel.data = {
            getAxisValue(msg, this->axis_x) * this->axis_speed * multiplier,
            getAxisValue(msg, this->axis_y) * this->axis_speed * multiplier,
            getAxisValue(msg, this->axis_z) * this->axis_speed * multiplier
        };

        // Calculate new gripper position
        // <0: closing, 0: stopped, >0: opening
        gripper.data = this->last_gripper.data;
        int gripper_direction = getButtonValue(msg, this->gripper_open_button) - getButtonValue(msg, this->gripper_close_button);
        if (gripper_direction) {
            // Direction is non-zero, therefore we need to move

            // Find current time
            std::chrono::steady_clock::time_point t = std::chrono::steady_clock::now();

            // If the gripper was already moving, then we can change the position this time step
            // If not, we need to note down the time and wait until the next time step to calculate how far to move
            if (this->gripper_moving) {
                // Calculate time since last gripper update
                std::chrono::duration<double> delta = t - this->last_time;

                // Apply the gripper velocity over this time
                // I don't want to blindly trust that the button value is normalized, so we extract the sign
                // An argument could be made that I shouldn't trust the joystick axes values either, but I don't think that is as big of a risk
                // As well, unlike the joystick axes the buttons are very easy to normalize
                gripper.data[0] = std::clamp(
                        gripper.data[0] += delta.count() * gripper_speed_converted * multiplier * boost::math::sign(gripper_direction),
                        this->gripper_max,
                        this->gripper_min
                );
            }
            else {
                // Gripper is supposed to be moving, but we are not guaranteed to have a correct last_time we can use to calculate the distance
                // Therefore we need to skip this step, but we can flag that we're supposed to be moving
                // Note that we do not have an extra step once the button is released, so this step is truly skipped
                this->gripper_moving = true;
            }

            // Save the current time
            this->last_time = t;
        }
        else {
            // The gripper is not being moved, change the flag to false
            this->gripper_moving = false;
        }

        // Publish the new values
        this->sendValues(vel, gripper);
    } else if (this->movement_enabled) {
        // Deadman switch no longer engaged, stop movement
        this->gripper_moving = false;
        this->sendValues(ZERO_VEL, this->last_gripper);
        this->movement_enabled = false; // Needs to be after sendValues since that sets movement_enabled = true
    }
}

void JoystickTeleopNode::sendValues(const std_msgs::msg::Float64MultiArray& vel, const std_msgs::msg::Float64MultiArray& gripper) {
    this->vel_publisher->publish(vel);
    this->gripper_publisher->publish(gripper);

    this->movement_enabled = true;
    this->last_vel = vel;
    this->last_gripper = gripper;
}

void JoystickTeleopNode::initializeParameters() {
    /*  Regex to apply to parameter list:
            deadman_button
            slow_button
            gripper_open_button
            gripper_close_button
            axis_x
            axis_y
            axis_z
            axis_speed
            gripper_speed
            slow_modifier
            gripper_min
            gripper_max
            vel_topic
            joy_topic
            gripper_topic
        Find:
            \s*(\w+)$
        Replace:
            rcl_interfaces::msg::ParameterDescriptor $1_d;
            $1_d.name = "$1";
            const auto& [$1_default, $1_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at\($1_d.name\);
            $1_d.description = $1_description;
            $1_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_;
            $1_d.read_only = true;
            $1_d.dynamic_typing = false;
            this->declare_parameter\($1_d.name, boost::get<???>\($1_default\), $1_d\);
            this->$1 = this->get_parameter\("$1"\).as_;\n
     */

    rcl_interfaces::msg::ParameterDescriptor deadman_button_d;
    deadman_button_d.name = "deadman_button";
    const auto& [deadman_button_default, deadman_button_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(deadman_button_d.name);
    deadman_button_d.description = deadman_button_description;
    deadman_button_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    deadman_button_d.read_only = true;
    deadman_button_d.dynamic_typing = false;
    this->declare_parameter(deadman_button_d.name, boost::get<int>(deadman_button_default), deadman_button_d);
    this->deadman_button = this->get_parameter("deadman_button").as_int();

    rcl_interfaces::msg::ParameterDescriptor slow_button_d;
    slow_button_d.name = "slow_button";
    const auto& [slow_button_default, slow_button_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(slow_button_d.name);
    slow_button_d.description = slow_button_description;
    slow_button_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    slow_button_d.read_only = true;
    slow_button_d.dynamic_typing = false;
    this->declare_parameter(slow_button_d.name, boost::get<int>(slow_button_default), slow_button_d);
    this->slow_button = this->get_parameter("slow_button").as_int();

    rcl_interfaces::msg::ParameterDescriptor gripper_open_button_d;
    gripper_open_button_d.name = "gripper_open_button";
    const auto& [gripper_open_button_default, gripper_open_button_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(gripper_open_button_d.name);
    gripper_open_button_d.description = gripper_open_button_description;
    gripper_open_button_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    gripper_open_button_d.read_only = true;
    gripper_open_button_d.dynamic_typing = false;
    this->declare_parameter(gripper_open_button_d.name, boost::get<int>(gripper_open_button_default), gripper_open_button_d);
    this->gripper_open_button = this->get_parameter("gripper_open_button").as_int();

    rcl_interfaces::msg::ParameterDescriptor gripper_close_button_d;
    gripper_close_button_d.name = "gripper_close_button";
    const auto& [gripper_close_button_default, gripper_close_button_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(gripper_close_button_d.name);
    gripper_close_button_d.description = gripper_close_button_description;
    gripper_close_button_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    gripper_close_button_d.read_only = true;
    gripper_close_button_d.dynamic_typing = false;
    this->declare_parameter(gripper_close_button_d.name, boost::get<int>(gripper_close_button_default), gripper_close_button_d);
    this->gripper_close_button = this->get_parameter("gripper_close_button").as_int();

    rcl_interfaces::msg::ParameterDescriptor axis_x_d;
    axis_x_d.name = "axis_x";
    const auto& [axis_x_default, axis_x_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(axis_x_d.name);
    axis_x_d.description = axis_x_description;
    axis_x_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    axis_x_d.read_only = true;
    axis_x_d.dynamic_typing = false;
    this->declare_parameter(axis_x_d.name, boost::get<int>(axis_x_default), axis_x_d);
    this->axis_x = this->get_parameter("axis_x").as_int();

    rcl_interfaces::msg::ParameterDescriptor axis_y_d;
    axis_y_d.name = "axis_y";
    const auto& [axis_y_default, axis_y_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(axis_y_d.name);
    axis_y_d.description = axis_y_description;
    axis_y_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    axis_y_d.read_only = true;
    axis_y_d.dynamic_typing = false;
    this->declare_parameter(axis_y_d.name, boost::get<int>(axis_y_default), axis_y_d);
    this->axis_y = this->get_parameter("axis_y").as_int();

    rcl_interfaces::msg::ParameterDescriptor axis_z_d;
    axis_z_d.name = "axis_z";
    const auto& [axis_z_default, axis_z_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(axis_z_d.name);
    axis_z_d.description = axis_z_description;
    axis_z_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    axis_z_d.read_only = true;
    axis_z_d.dynamic_typing = false;
    this->declare_parameter(axis_z_d.name, boost::get<int>(axis_z_default), axis_z_d);
    this->axis_z = this->get_parameter("axis_z").as_int();

    rcl_interfaces::msg::ParameterDescriptor axis_speed_d;
    axis_speed_d.name = "axis_speed";
    const auto& [axis_speed_default, axis_speed_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(axis_speed_d.name);
    axis_speed_d.description = axis_speed_description;
    axis_speed_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    axis_speed_d.read_only = true;
    axis_speed_d.dynamic_typing = false;
    this->declare_parameter(axis_speed_d.name, boost::get<double>(axis_speed_default), axis_speed_d);
    this->axis_speed = this->get_parameter("axis_speed").as_double();

    rcl_interfaces::msg::ParameterDescriptor gripper_speed_d;
    gripper_speed_d.name = "gripper_speed";
    const auto& [gripper_speed_default, gripper_speed_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(gripper_speed_d.name);
    gripper_speed_d.description = gripper_speed_description;
    gripper_speed_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    gripper_speed_d.read_only = true;
    gripper_speed_d.dynamic_typing = false;
    this->declare_parameter(gripper_speed_d.name, boost::get<double>(gripper_speed_default), gripper_speed_d);
    this->gripper_speed = this->get_parameter("gripper_speed").as_double();

    rcl_interfaces::msg::ParameterDescriptor slow_modifier_d;
    slow_modifier_d.name = "slow_modifier";
    const auto& [slow_modifier_default, slow_modifier_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(slow_modifier_d.name);
    slow_modifier_d.description = slow_modifier_description;
    slow_modifier_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    slow_modifier_d.read_only = true;
    slow_modifier_d.dynamic_typing = false;
    this->declare_parameter(slow_modifier_d.name, boost::get<double>(slow_modifier_default), slow_modifier_d);
    this->slow_modifier = this->get_parameter("slow_modifier").as_double();

    rcl_interfaces::msg::ParameterDescriptor gripper_min_d;
    gripper_min_d.name = "gripper_min";
    const auto& [gripper_min_default, gripper_min_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(gripper_min_d.name);
    gripper_min_d.description = gripper_min_description;
    gripper_min_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    gripper_min_d.read_only = true;
    gripper_min_d.dynamic_typing = false;
    this->declare_parameter(gripper_min_d.name, boost::get<double>(gripper_min_default), gripper_min_d);
    this->gripper_min = this->get_parameter("gripper_min").as_double();

    rcl_interfaces::msg::ParameterDescriptor gripper_max_d;
    gripper_max_d.name = "gripper_max";
    const auto& [gripper_max_default, gripper_max_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(gripper_max_d.name);
    gripper_max_d.description = gripper_max_description;
    gripper_max_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    gripper_max_d.read_only = true;
    gripper_max_d.dynamic_typing = false;
    this->declare_parameter(gripper_max_d.name, boost::get<double>(gripper_max_default), gripper_max_d);
    this->gripper_max = this->get_parameter("gripper_max").as_double();

    rcl_interfaces::msg::ParameterDescriptor vel_topic_d;
    vel_topic_d.name = "vel_topic";
    const auto& [vel_topic_default, vel_topic_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(vel_topic_d.name);
    vel_topic_d.description = vel_topic_description;
    vel_topic_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    vel_topic_d.read_only = true;
    vel_topic_d.dynamic_typing = false;
    this->declare_parameter(vel_topic_d.name, boost::get<std::string>(vel_topic_default), vel_topic_d);
    this->vel_topic = this->get_parameter("vel_topic").as_string();

    rcl_interfaces::msg::ParameterDescriptor gripper_topic_d;
    gripper_topic_d.name = "gripper_topic";
    const auto& [gripper_topic_default, gripper_topic_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(gripper_topic_d.name);
    gripper_topic_d.description = gripper_topic_description;
    gripper_topic_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    gripper_topic_d.read_only = true;
    gripper_topic_d.dynamic_typing = false;
    this->declare_parameter(gripper_topic_d.name, boost::get<std::string>(gripper_topic_default), gripper_topic_d);
    this->gripper_topic = this->get_parameter("gripper_topic").as_string();

    rcl_interfaces::msg::ParameterDescriptor joy_topic_d;
    joy_topic_d.name = "joy_topic";
    const auto& [joy_topic_default, joy_topic_description] = JoystickTeleopNode::DEFAULT_PARAMETERS.at(joy_topic_d.name);
    joy_topic_d.description = joy_topic_description;
    joy_topic_d.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    joy_topic_d.read_only = true;
    joy_topic_d.dynamic_typing = false;
    this->declare_parameter(joy_topic_d.name, boost::get<std::string>(joy_topic_default), joy_topic_d);
    this->joy_topic = this->get_parameter("joy_topic").as_string();
}

double getAxisValue(const sensor_msgs::msg::Joy::ConstSharedPtr& msg, const size_t axis) {
    // Ensure axis exists, return 0.0 if not. Note axis is size_t so always >= 0
    return (msg->axes.size() > axis) ? msg->axes[axis] : 0.0;
}

int getButtonValue(const sensor_msgs::msg::Joy::ConstSharedPtr& msg, const size_t button) {
    // Ensure button exists, return 0 if not. Note button is size_t so always >= 0
    return (msg->buttons.size() > button) ? msg->buttons[button] : 0;
}