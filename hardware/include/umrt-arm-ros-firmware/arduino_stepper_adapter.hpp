#ifndef UMRT_ARM_ROS_FIRMWARE_ARDUINOSTEPPERCONTROLLER_HPP
#define UMRT_ARM_ROS_FIRMWARE_ARDUINOSTEPPERCONTROLLER_HPP

#include "umrt-arm-ros-firmware/stepper_adapter.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <thread>
#include <umrt-arm-firmware-lib/arduino_stepper_controller.hpp>

/**
 * Adapter class to interface a @ref StepperController with a ros2_control
 * hardware_interface.
 */
class ArduinoStepperAdapter : public StepperAdapter {
public:
    /**
    * Initializes an ArduinoStepperAdapter, but does not connect it.
    *
    * @param NUM_JOINTS the number of joints
    * @param query_period the time to wait between controller queries for position, velocity, etc.
    */
    ArduinoStepperAdapter(
            const std::size_t NUM_JOINTS,
            const std::chrono::duration<int64_t, std::milli>& query_period
    );

    ~ArduinoStepperAdapter() override;

    /**
    * Connect to an Arduino running the Stepper Controller program.
    *
    * @param device the path to the serial device connected to the Arduino
    * @param baud_rate baud rate to use for the Firmata connection
    */
    void connect(const std::string device, const int baud_rate) override;

    /**
     * Disconnect from the Arduino and close polling loops.
     */
    void disconnect() override;

    /**
     * Write the current contents of the command registers, which are accessible
     * through @ref getCommandRef, to the Stepper Controller program.
     */
    void setValues() override;

protected:
    /**
     * The ArduinoStepperController which implements the functionality exposed by this
     * ArduinoStepperAdapter.
     */
    ArduinoStepperController controller;

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

#endif //UMRT_ARM_ROS_FIRMWARE_ARDUINOSTEPPERCONTROLLER_HPP
