#ifndef SYSEX_COMMANDS_H
#define SYSEX_COMMANDS_H

// TODO: Consider adding a keep-alive
// Note: Only allowed to use 0x00-0x0F, beyond that would have to nest commands
enum SysexCommands : uint8_t {
  ECHO = 0x00, // Respond back with payload
  SET_SPEED,   // Set the speed of a motor. First byte is motor ID, next 4 bytes are signed speed.
  GET_SPEED    // Get the current speed of a motor. First byte is motor ID.
};

#endif
