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
