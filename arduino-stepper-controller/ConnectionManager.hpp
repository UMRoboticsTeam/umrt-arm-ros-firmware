#ifndef CONNECTION_MANAGER_HPP
#define CONNECTION_MANAGER_HPP

// Pololu High-Power Stepper Motor Driver Library for Arduino
// https://github.com/pololu/high-power-stepper-driver-arduino/tree/master
// For interacting with https://www.pololu.com/product/3730
#include <HighPowerStepperDriver.h>

#define MAX_DRIVERS 6

class ConnectionManager {
public:
  static void init();

  uint8_t add_driver(uint8_t step_pin, uint8_t dir_pin, uint8_t cs_pin, uint16_t current);

  HighPowerStepperDriver* get_driver(uint8_t index);

  uint8_t get_step_pin(uint8_t index);

  uint8_t get_dir_pin(uint8_t index);

  ~ConnectionManager();

private:
  HighPowerStepperDriver* drivers[MAX_DRIVERS];
  uint8_t step_pins[MAX_DRIVERS];
  uint8_t dir_pins[MAX_DRIVERS];
  uint8_t num_drivers = 0;
};

#endif
