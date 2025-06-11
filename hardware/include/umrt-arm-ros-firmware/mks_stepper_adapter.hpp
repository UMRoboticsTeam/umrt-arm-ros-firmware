#ifndef UMRT_ARM_ROS_FIRMWARE_MKSSTEPPERCONTROLLER_HPP
#define UMRT_ARM_ROS_FIRMWARE_MKSSTEPPERCONTROLLER_HPP

#include "umrt-arm-ros-firmware/stepper_adapter.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>

#include <boost/bimap.hpp>

#include <umrt-arm-firmware-lib/mks_stepper_controller.hpp>

/**
 * Adapter class to interface an @ref MksStepperController with a ros2_control
 * hardware_interface.
 *
 * Unlike @ref ArduinoStepperController, this class follows the RAII paradigm.
 */
class MksStepperAdapter : public StepperAdapter {
public:
    /**
    * Initializes an MksStepperAdapter.
    * The number of joints is inferred from the number of motor IDs provided.
    *
    * @param can_interface SocketCAN network interface corresponding to the CAN bus
    * @param motor_ids CAN IDs for the motor controller
    * @param query_period the time to wait between controller queries for position, velocity, etc.
    */
    MksStepperAdapter(
            const std::string& can_interface,
            const std::vector<uint16_t>& motor_ids,
            const std::chrono::duration<int64_t, std::milli>& query_period
    );

    ~MksStepperAdapter() override;

    /**
     * Write the current contents of the command registers, which are accessible
     * through @ref getCommandRef, to the Stepper Controller program.
     */
    void setValues() override;

protected:
    /**
     * The MksStepperController which implements the functionality exposed by this
     * MksStepperAdapter.
     */
    std::unique_ptr<MksStepperController> controller;

    /**
     * Thread used to run @ref poll indefinitely.
     */
    std::thread polling_thread;

    /**
     * Thread used to periodically query motor speed/position.
     */
    std::thread querying_thread;

    /**
     * Signal used to shutdown the polling threads.
     */
    std::atomic<bool> continue_polling = false;

    /**
     * Maps joint index to motor CAN IDs.
     */
    std::unique_ptr<boost::bimap<uint16_t, uint16_t>> motor_ids;

    /**
     * Method to indefinitely poll @ref controller for responses
     */
    void poll();

    /**
     * Queries the position and speed from @ref controller. Used as a callback to the wall querying_thread setup in @ref init.
     */
    void queryController();

    /**
     * Poll loop used to trigger motor queries.
     *
     * @param period Amount of time to wait in milliseconds between queries
     */
    void queryPoll(const std::chrono::milliseconds& period);
};

#endif //UMRT_ARM_ROS_FIRMWARE_MKSSTEPPERCONTROLLER_HPP
