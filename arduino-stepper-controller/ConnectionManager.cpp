#include "ConnectionManager.hpp"
#include <SPI.h>

static void ConnectionManager::init() {
    SPI.begin();
}

uint8_t ConnectionManager::add_driver(uint8_t step_pin, uint8_t dir_pin, uint8_t cs_pin, uint16_t current, HPSDStepMode step_mode) {
    // Create the HighPowerStepperDriver object, and save it and its info to the arrays
    HighPowerStepperDriver* sd = new HighPowerStepperDriver();
    this->drivers[this->num_drivers] = sd;
    this->step_pins[this->num_drivers] = step_pin;
    this->dir_pins[this->num_drivers] = dir_pin;

    sd->setChipSelectPin(cs_pin);

    // Drive the STEP and DIR pins low initially.
    pinMode(step_pin, OUTPUT);
    digitalWrite(step_pin, LOW);
    pinMode(dir_pin, OUTPUT);
    digitalWrite(dir_pin, LOW);

    // Give the driver some time to power up.
    delay(1);

    // TODO: Might want to wait until non-zero read from driver SPI connection

    // Reset the driver to its default settings and clear latched status
    // conditions.
    sd->resetSettings();
    sd->clearStatus();

    // Set driver settings and enable it
    sd->setDecayMode(HPSDDecayMode::AutoMixed); // As recommended by HighPowerStepperMotorDriver example
    sd->setCurrentMilliamps36v4(current);
    sd->setStepMode(step_mode);
    sd->enableDriver();

    return this->num_drivers++;
}

HighPowerStepperDriver* ConnectionManager::get_driver(uint8_t index) {
    return index < this->num_drivers ? this->drivers[index] : NULL;
}

uint8_t ConnectionManager::get_step_pin(uint8_t index) {
    return index < this->num_drivers ? this->step_pins[index] : NULL;
}

uint8_t ConnectionManager::get_dir_pin(uint8_t index) {
    return index < this->num_drivers ? this->dir_pins[index] : NULL;
}

ConnectionManager::~ConnectionManager() {
    for (int i = 0; i < this->num_drivers; ++i) {
        delete this->drivers[i];
    }
    this->num_drivers = 0;
}
