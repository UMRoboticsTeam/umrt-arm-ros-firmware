#ifndef STEPPER_ADAPTER_HPP
#define STEPPER_ADAPTER_HPP

#include <cstddef>
#include <exception>
#include <thread>
#include <vector>

#include <rclcpp/node.hpp>

#include "StepperController.h"

/**
 * Adapter class to interface a @ref StepperController with a ros2_control
 * hardware_interface.
 */
class StepperAdapter {
public:
    /**
    * Initializes this StepperAdapter. Must be called before any other method,
    * and only once.
    *
    * The number of joints available must be specified here in order to
    * facilitate initialization such as array sizing.
    *
    * @param NUM_JOINTS the number of joints
    * @param parentNode the Node to use for creating WallTimers
    * @param queryPeriod the time to wait between controller queries for position, velocity, etc.
    */
    void init(
            const std::size_t NUM_JOINTS,
            rclcpp::Node& parentNode,
            const std::chrono::duration<int64_t, std::milli>& queryPeriod
    );

    /**
    * Connect to an Arduino running the Stepper Controller program.
    *
    * @param device the path to the serial device connected to the Arduino
    * @param baud_rate baud rate to use for the Firmata connection
    */
    void connect(const std::string device, const int baud_rate);

    /**
     * Disconnect from the Arduino.
     */
    void disconnect();

    /**
     * Write the current contents of the command registers, which are accessible
     * through @ref getCommandRef, to the Stepper Controller program.
     */
    void setValues();

    /**
     * Read the current values from the Stepper Controller program into the
     * position and velocity registers, which are accessible through
     * @ref getPositionRef and @ref getVelocityRef respectively.
     */
    void readValues();

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
     * Thread used to run @ref poll indefinitely.
     */
    std::thread polling_thread;

    /**
     * Method to indefinitely poll @ref controller for responses
     */
    [[noreturn]] void poll();

    /**
     * Queries the position and speed from @ref controller. Used as a callback to the wall timer setup in @ref init.
     */
    void queryController();

    /**
     * Helper function to ensure that this StepperAdapter has been initialized
     * before attempting operations.
     *
     * @throws std::logic_error if this StepperAdapter has not been initialized
     *                          at the time of calling
     */
    void initializedCheck();

private:
    /**
     * Flag which indicates whether @ref init has completed.
     */
    bool initialized = false;
};

#endif // STEPPER_ADAPTER_HPP