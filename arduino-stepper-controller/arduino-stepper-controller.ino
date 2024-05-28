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

#define BUFF_SIZE 256

#define STEP_1 3
#define DIR_1 2
#define CS_1 4
#define CURRENT_1 2000
#define FULLROT_1 200


void echo(byte command, byte argc, byte* argv);
void set_speed(byte command, byte argc, byte* argv);
void get_speed(byte command, byte argc, byte* argv);

ConnectionManager manager;
MoToStepper step_1(FULLROT_1, STEPDIR);

void setup() {
  // put your setup code here, to run once:
  ConnectionManager::init();

  // Setup Firmata
  Firmata.begin(SERIAL_SPEED);
  Firmata.attach(SysexCommands::ECHO, echo);
  Firmata.attach(SysexCommands::GET_SPEED, get_speed);
  Firmata.attach(SysexCommands::SET_SPEED, set_speed);
 
  manager.add_driver(STEP_1, DIR_1, CS_1, CURRENT_1);

  step_1.attach(STEP_1, DIR_1);

  // =========TESTING=========
  step_1.setSpeed(10000); // 1000 RPM
  step_1.rotate(1);
  // =========================
}

void loop() {
  uint8_t ack[] = {0x6E };
  //Firmata.sendSysex(SysexCommands::ECHO, 1, ack);
  while (Firmata.available())
  {
    Firmata.processInput();
  }
}

// Payload: anything
void echo(byte command, byte argc, byte* argv){
  Firmata.sendSysex(SysexCommands::ECHO, argc, argv);
}

// Payload:
// [ 
//    uint8_t motorID,
//    int32_t speed     // Big endian; units of 10*rpm
// ]
void set_speed(byte command, byte argc, byte* argv){
  // Check arguments
  if (argc != 2) { return; }

  // Calculate speed
  int32_t speed = argv[1] << 24 | argv[2] << 16 | argv[1] << 8 | argv[0];

  // TODO: Finish setting up for additional motors
  step_1.setSpeed(speed);
}

// Payload: 
// [ uint8_t motorID ]
//
// Responds:
// [ 
//    uint8_t motorID,
//    int32_t speed     // Big endian; units of 10*rpm
// ]
void get_speed(byte command, byte argc, byte* argv){
  // Check arguments
  if (argc != 2) { return; }

  // TODO: Write
}
