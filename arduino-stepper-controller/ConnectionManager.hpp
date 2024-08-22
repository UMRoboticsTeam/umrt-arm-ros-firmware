#ifndef CONNECTION_MANAGER_HPP
#define CONNECTION_MANAGER_HPP

// Pololu High-Power Stepper Motor Driver Library for Arduino
// https://github.com/pololu/high-power-stepper-driver-arduino/
// For interacting with https://www.pololu.com/product/3730
#include <HighPowerStepperDriver.h>

#define MAX_DRIVERS 6

class ConnectionManager {
public:
    /**
     * Initializes the required Arduino modules for ConnectionManager operation. Must be called before the
     * construction of a ConnectionManager.
     */
    static void init();

    /**
     * Creates a @ref HighPowerStepperDriver to be managed by this ConnectionManager.
     *
     * @param step_pin the Arduino pin the driver's STEP pin is connected to
     * @param dir_pin the Arduino pin the driver's DIR pin is connected to
     * @param cs_pin the Arduino pin the driver's SCS pin is connected to
     * @param current the maximum current this driver is allowed to supply to the connected motor
     * @param step_mode the microstepping mode to use with this driver
     * @return the index assigned to this driver
     */
    uint8_t add_driver(uint8_t step_pin, uint8_t dir_pin, uint8_t cs_pin, uint16_t current, HPSDStepMode step_mode);

    /**
     * Retrieves a pointer to the @ref HighPowerStepperDriver at the specified index.
     *
     * @param index index of the requested driver
     * @return a pointer to the @ref HighPowerStepperDriver, or `NULL`
     */
    HighPowerStepperDriver* get_driver(uint8_t index);

    /**
     * Retrieves the Arduino pin associated with the specified driver's STEP pin.
     *
     * @param index index of the driver to examine
     * @return the Arduino pin the driver's STEP pin is connected to
     */
    uint8_t get_step_pin(uint8_t index);

    /**
     * Retrieves the Arduino pin associated with the specified driver's DIR pin.
     *
     * @param index index of the driver to examine
     * @return the Arduino pin the driver's DIR pin is connected to
     */
    uint8_t get_dir_pin(uint8_t index);

    /**
     * Destroys this ConnectionManager and associated @ref HighPowerStepperDrivers.
     */
    ~ConnectionManager();

private:
    /**
     * Array holding pointers to dynamically-allocated @ref HighPowerStepperDrivers.
     */
    HighPowerStepperDriver* drivers[MAX_DRIVERS];

    /**
     * The STEP pin associated with the co-indexed @ref HighPowerStepperDriver stored in @ref drivers
     */
    uint8_t step_pins[MAX_DRIVERS];

    /**
     * The DIR pin associated with the co-indexed @ref HighPowerStepperDriver stored in @ref drivers
     */
    uint8_t dir_pins[MAX_DRIVERS];

    /**
     * The number of @ref HighPowerStepperDriver%s stored in @ref drivers
     */
    uint8_t num_drivers = 0;
};

#endif
