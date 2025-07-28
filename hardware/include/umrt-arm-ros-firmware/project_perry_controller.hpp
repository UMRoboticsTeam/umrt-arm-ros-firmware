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
 * Adapter class to interface an @ref MksStepperController with a ros2_control hardware_interface, with joints handled as
 * per the Project Perry mechanical implementation.
 *
 * The reason this class is not arm-agnostic is that Project Perry has a differential wrist, which requires special handling
 * when executing motor commands.
 *
 * Unlike @ref ArduinoStepperController, this class follows the RAII paradigm.
 */
class ProjectPerryController : public StepperAdapter {
public:
    /**
    * Initializes a ProjectPerryController.
    * The number of joints is inferred from the number of motor IDs provided.
    *
    * @param can_interface SocketCAN network interface corresponding to the CAN bus
    * @param joint_infos info needed to process joint feedback, requires motor_id and reduction_factor
    * @param position_commandable whether position commands can be accepted
    * @param default_speed speed to use when a position command is sent without specifying a velocity
    * @param query_period time to wait between controller queries for position, velocity, etc.
    * @param logger rclcpp::Logger to use for ROS log messages
    */
    ProjectPerryController(
            const std::string& can_interface,
            const std::vector<JointInfo>& joint_infos,
            const bool position_commandable,
            const double default_speed,
            const std::chrono::duration<int64_t, std::milli>& query_period,
            rclcpp::Logger& logger
    );

    ~ProjectPerryController() override;

    /** Does nothing. */
    void connect(const std::string device, const int baud_rate) override;

    /** Does nothing. */
    void disconnect() override;

    /**
     * Write the current contents of the command registers, which are accessible
     * through @ref getCommandRef, to the Stepper Controller program.
     */
    void setValues() override;

    /** Whether position commands can be accepted. */
    const bool position_commandable;

protected:
    /** Speed used when a position command is sent without specifying a velocity. */
    const double default_speed;

    /** Logger used for sending ROS log messages. */
    rclcpp::Logger& logger;

    /** The MksStepperController which is used to command motors. */
    std::unique_ptr<MksStepperController> controller;

    /** Thread used to run @ref poll indefinitely. */
    std::thread polling_thread;

    /** Thread used to periodically query motor speed/position. */
    std::thread querying_thread;

    /** Signal used to shutdown the polling threads. */
    std::atomic<bool> continue_polling = false;

    /** Maps joint index to motor CAN IDs. */
    std::unique_ptr<boost::bimap<uint16_t, uint16_t>> motor_ids;

    /** Maps joint index to reduction ratio factor. */
    std::unique_ptr<std::unordered_map<uint16_t, double>> reductions;

    /** Maps motor CAN IDs to last commanded position, used for debug logging. */
    std::unique_ptr<std::unordered_map<uint16_t, int32_t>> last_motor_commands;

    /** Method to indefinitely poll @ref controller for responses */
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
