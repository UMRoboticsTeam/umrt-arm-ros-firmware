#ifndef UMRT_ARM_ROS_FIRMWARE_STEPPER_ADAPTER_HPP
#define UMRT_ARM_ROS_FIRMWARE_STEPPER_ADAPTER_HPP

#include <cstddef>
#include <exception>
#include <mutex>
#include <thread>
#include <vector>

#include <rclcpp/node.hpp>

#include <umrt-arm-firmware-lib/StepperController.h>

/**
 * Adapter class to interface a @ref StepperController with a ros2_control
 * hardware_interface.
 * TODO: Make docs more generic
 */
class StepperAdapter {
public:
    /**
    * Creates a StepperAdapter.
    *
    * The number of joints available must be specified here in order to
    * facilitate initialization such as array sizing.
    *
    * @param NUM_JOINTS the number of joints
    */
    explicit StepperAdapter(const std::size_t NUM_JOINTS);

    virtual ~StepperAdapter();

    /**
    * Connect to an Arduino running the Stepper Controller program.
    *
    * @param device the path to the serial device connected to the Arduino
    * @param baud_rate baud rate to use for the Firmata connection
    */
    virtual void connect(const std::string device, const int baud_rate) = 0;

    /**
     * Disconnect from the Arduino and close polling loops.
     */
    virtual void disconnect() = 0;

    /**
     * Write the current contents of the command registers, which are accessible
     * through @ref getCommandRef, to the Stepper Controller program.
     */
    virtual void setValues() = 0;

    /**
     * Read the current values from the Stepper Controller program into the
     * position and velocity registers, which are accessible through
     * @ref getPositionRef and @ref getVelocityRef respectively.
     */
    virtual void readValues() = 0;

    /**
     * Exposes the position register for the specified joint. Refreshed by
     * @ref readValues.
     *
     * @param index index of the joint to access
     * @return a reference to the joint's current position
     */
    double& getPositionRef(std::size_t index);

    /**
     * Exposes the position register for the specified joint. Refreshed by
     * @ref readValues.
     *
     * @param index index of the joint to access
     * @return a reference to the joint's current velocity
     */
    double& getVelocityRef(std::size_t index);

    /**
     * Exposes the position register for the gripper. Refreshed by
     * @ref readValues.
     *
     * @return a reference to the gripper's current position
     */
    double& getGripperPositionRef();

    /**
     * Exposes the velocity command register for the specified joint. Pushed by
     * @ref setValues.
     *
     * @param index index of the joint to access
     * @return a reference to the joint's requested velocity
     */
    double& getCommandRef(std::size_t index);

    /**
     * Exposes the position command register for the gripper. Pushed by
     * @ref setValues.
     *
     * @return a reference to the gripper's requested position
     */
    double& getGripperPositionCommandRef();

protected:
    /**
     * The StepperController which implements the functionality exposed by this
     * StepperAdapter.
     */
    StepperController controller;

    // It is very important that the size of these vectors is not changed after init has been called, since we need
    // element pointer stability. Unfortunately, we also need element mutability so const vectors can't be used. As
    // well, the size cannot be determined at compile-time so std::arrays can't be used either. Random-access is
    // required, so std::lists are not a good option either.
    // Note that while it may seem a little silly to expose references to the vector elements, but hide the vectors
    // themselves, this is an important way to prevent the invalidation of references by changing the vectors' sizes
    std::vector<double> commands;
    std::vector<double> positions;
    std::vector<double> velocities;
    double gripper_position;
    double cmd_gripper_pos;

    /**
     * The joint positions to be transferred into @ref positions during the next @ref readValues call.
     * @ref positions_buffer_mx must be acquired before interacting with this vector.
     */
    std::vector<double> positions_buffer;

    /**
     * The joint positions to be transferred into @ref positions during the next @ref readValues call.
     * @ref velocities_buffer_mx must be acquired before interacting with this vector.
     */
    std::vector<double> velocities_buffer;

    /**
     * Lock to protect @ref positions_buffer.
     */
    std::mutex positions_buffer_mx;

    /**
     * Lock to protect @ref velocities_buffer.
     */
    std::mutex velocities_buffer_mx;
};

#endif // UMRT_ARM_ROS_FIRMWARE_STEPPER_ADAPTER_HPP