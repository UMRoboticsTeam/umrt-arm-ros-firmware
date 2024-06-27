#include "ConnectionManager.hpp"
#include "SYSEX_COMMANDS.h"

// MobaTools
// https://github.com/MicroBahner/MobaTools/tree/master
// For convenient driving stepper motors
#include <MobaTools.h>

// Firmata 
// https://github.com/firmata/arduino
// For communication between USB master and Arduino
#include <Firmata.h>

#define SERIAL_SPEED 57600

#define MAX_STEPPERS 3

struct MotorInfo {
  const uint8_t dir_pin;
  const uint8_t step_pin;
  const uint8_t cs_pin;
  const uint16_t current;
  const long full_rot_steps;
};
constexpr MotorInfo motor_infos[] = {
  {3, 2, 4, 2800, 200},
  {6, 5, 7, 2800, 200},
  {9, 8, 10, 2800, 200}
};
constexpr uint8_t num_motors = sizeof(motor_infos) / sizeof(MotorInfo); // Gross, but no type_traits support for Arduino yet

// Done as macro so don't have to create versions for int8_t, int16_t etc.
#define SGN(X) ((X > 0) - (X < 0))

constexpr HPSDStepMode STEP_MODE = HPSDStepMode::MicroStep1; // Full steps

void echo_string(char* str);
void echo(byte argc, byte* argv);
void set_speed(byte argc, byte* argv);
void get_speed(byte argc, byte* argv);
void send_step(byte argc, byte* argv);
int32_t decode_32(byte argc, byte* argv);
int16_t decode_16(byte argc, byte* argv);
void pack_32(byte* arr, int32_t integer);
void pack_16(byte* arr, int16_t integer);
void sysex_handler(byte command, byte argc, byte* argv);

ConnectionManager manager;
MoToStepper* steppers[MAX_STEPPERS];
//steppers[0] = new MoToStepper(FULLROT_0, STEPDIR);
//steppers[1] = new MoToStepper(FULLROT_1, STEPDIR);
//steppers[2] = new MoToStepper(FULLROT_2, STEPDIR);

void setup() {
  // Setup ConnectionManager
  ConnectionManager::init();

  // Setup motor drivers
  for (uint8_t i = 0; i < num_motors; ++i) {
    MotorInfo mi = motor_infos[i];
    steppers[i] = new MoToStepper(mi.full_rot_steps, STEPDIR);
    manager.add_driver(mi.step_pin, mi.dir_pin, mi.cs_pin, mi.current, STEP_MODE);
    steppers[i]->attach(mi.step_pin, mi.dir_pin);
  }

  // Setup Firmata
  Firmata.setFirmwareVersion(FIRMATA_FIRMWARE_MAJOR_VERSION, FIRMATA_FIRMWARE_MINOR_VERSION);
  Firmata.attach(STRING_DATA, echo_string);
  Firmata.attach(SysexCommands::ECHO, sysex_handler); // Contrary to what one might expect, if the command parameter isn't well-known, it is ignored and sysex_handler is used for all unknown messages
  Firmata.begin(SERIAL_SPEED);

  // =========TESTING=========
  steppers[0]->setSpeed(10); // 1 RPM
  steppers[0]->rotate(1);
  // =========================
}

void loop() {
  //uint8_t ack[] = {0x6E };
  //Firmata.sendSysex(SysexCommands::ECHO, 1, ack);
  while (Firmata.available())
  {
    Firmata.processInput();
  }
}

// Respond back with payload
// In:  [ payload (anything) ]
// Out: [ payload (anything) ]
void echo_string(char* str) {
  Firmata.sendString(str);
}

// Respond back with payload
// In:  [ payload (anything) ]
// Out: [ payload (anything) ]
void echo(byte argc, byte* argv){
  char buff[22];
  sprintf(buff, "%d,%d,%d,%d,%d,%d", argv[0], argv[1], argv[2], argv[3], argv[4], argv[5]);
  Firmata.sendString(buff);
  
  Firmata.sendSysex(SysexCommands::ECHO, argc, argv);
}

// Set the speed of a motor
// In:  [ motor_id (uint8_t), speed (int16_t)]
// Out: [ motor_id (uint8_t), speed (int16_t)]
void set_speed(byte argc, byte* argv){
  // Check arguments
  //if (argc != 3) { return; }

  // Get motor ID
  uint8_t motor = argv[0];
  if (motor >= num_motors) { return; }

  // Decode speed
  int16_t speed = decode_16(argc - 1, argv + 1);

  // Send back some debugging stuff
  char buff[22];
  // PRId16 is macro for int16_t format specifier
  sprintf(buff, "%d: {%d, %d}=%" PRId16, motor, argv[1], argv[2], speed);
  Firmata.sendString(buff);
  
  // TODO: Finish setting up for additional motors
  steppers[motor]->setSpeed(abs(speed));
  steppers[motor]->rotate(SGN(speed));
}

// Get the current speed of a motor
// In:  [ motor_id (uint8_t) ]
// Out: [ motor_id (uint8_t), speed (int16_t)]
void get_speed(byte argc, byte* argv){
  // Check arguments
  if (argc != 2) { return; }

  // TODO: Write
}

// Move the motor a specific number of steps (negative speed allowed)
// Scheduled steps may be overridden by other commands
// In:  [ motor_id (uint8_t), num_steps (uint16_t), speed (int16_t) ]
// Out: [ motor_id (uint8_t), num_steps (uint16_t), speed (int16_t) ]
void send_step(byte argc, byte* argv) {
  // Check arguments
  //if (argc != 5) { return; }

  // Get motor ID
  uint8_t motor = argv[0];
  if (motor >= num_motors) { return; }

  // Decode num_steps and speed
  int16_t num_steps = decode_16(2, argv + 1);
  int16_t speed = decode_16(2, argv + 3);

  // Send back some debugging stuff
  char buff[39];
  // PRId16 is macro for int16_t format specifier
  sprintf(buff, "%d: {%d, %d, %d, %d}=%" PRId16 ", %" PRId16, motor, argv[1], argv[2], argv[3], argv[4], num_steps, speed);
  Firmata.sendString(buff);
  
  // TODO: Finish setting up for additional motors
  steppers[motor]->setSpeed(abs(speed));
  steppers[motor]->move(SGN(speed) * num_steps);
}

int32_t decode_32(byte argc, byte* argv){
    if (argc != 4) { return 0; }
    
    // Note: Shifts are casted so that all terms are 32-bits wide
    //       Shifts are casted to uint32_t instead of int32_t to ensure no weird sign stuff
    return (int32_t)argv[0] | argv[1] << (uint32_t)8 | argv[2] << (uint32_t)16 << argv[3] << (uint32_t)24;
}

int16_t decode_16(byte argc, byte* argv){
    if (argc != 2) { return 0; }
    
    // Note: Shift is casted to uint16_t instead of int16_t to ensure no weird sign stuff
    return (int16_t)argv[0] | argv[1] << (uint16_t)8;
}

void pack_32(byte* arr, int32_t integer){
    // Little-endian
    //
    // e.g. for 0xDEAD_BEEF:
    // 1101 1110 1010 1101 1011 1110 1110 1111
    // 3333 3333 2222 2222 1111 1111 0000 0000
    // packed = [ 0xEF, 0xBE, 0xAD, 0xDE ]
    arr[0] = integer & 0xFF; // bits [7, 0]
    arr[1] = integer >> 8 & 0xFF; // bits [15, 8]
    arr[2] = integer >> 16 & 0xFF; // bits [23, 16]
    arr[3] = integer >> 24 & 0xFF; // bits [31, 24]
}

void pack_16(byte* arr, int16_t integer){
    // Little-endian
    //
    // e.g. for 0xBEEF:
    // 1011 1110 1110 1111
    // 1111 1111 0000 0000
    // packed = [ 0xEF, 0xBE ]
    arr[0] = integer & 0xFF; // bits [7, 0]
    arr[1] = integer >> 8 & 0xFF; // bits [15, 8]
}

void sysex_handler(byte command, byte argc, byte* argv){
  // Since all input must be firmatified, argc must be even
  if (argc % 2) { return; }
  
  // Defirmatify data in place
  argc /= 2;
  for (int i = 0; i < argc; ++i) {
    argv[i] = argv[2*i] | argv[2*i + 1] << 7;
  }
  
  switch (command) {
  case SysexCommands::ECHO: echo(argc, argv); break;
  case SysexCommands::SET_SPEED: set_speed(argc, argv); break;
  case SysexCommands::GET_SPEED: get_speed(argc, argv); break;
  case SysexCommands::SEND_STEP: send_step(argc, argv); break;
  }
}
