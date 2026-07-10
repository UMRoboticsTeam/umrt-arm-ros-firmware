#ifndef UMRT_ARM_ROS_FIRMWARE_WHEELCONTROLLER_HPP
#define UMRT_ARM_ROS_FIRMWARE_WHEELCONTROLLER_HPP

#include "umrt-arm-ros-firmware/stepper_adapter.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>

#include <boost/bimap.hpp>

// #include <umrt-arm-firmware-lib/wheel_controller.hpp>

/**
 * Adapter class utilized for ros2_control hardware interface for 
 * command interface and state interfaces.
 */
class WheelAdapter {
public:
    /**
    * Initializes an WheelAdapter.
    * The number of joints is inferred from the number of motor IDs provided.
    */
    WheelAdapter(
        const std::size_t num_joints
    );

    ~WheelAdapter();

    /** Does nothing. */
    void connect(const std::string device, const int baud_rate);

    /** Does nothing. */
    void disconnect();

    /** Does nothing. */
    void writeValues();

    /** Does nothing. */
    void readValues();

    double& getVelocityRef(std::size_t index);
    double& getCommandRef(std::size_t index);
    double& getPositionRef(std::size_t index);

protected:
    
    //  Vector for velocity joint commands.
    std::vector<double> commands;

    //  Vector for joint positions.
    std::vector<double> positions;

    //  Vector for joint velocities.
    std::vector<double> velocities;

};

#endif //UMRT_ARM_ROS_FIRMWARE_WHEELCONTROLLER_HPP
