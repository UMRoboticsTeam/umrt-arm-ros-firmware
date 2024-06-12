from pyfirmata2 import Arduino, util
import time

COM_PORT = 'COM3'
SYSEX_COMMAND_ECHO = 0x00
SYSEX_COMMAND_SET_SPEED = 0x01

def pack_32(integer):
    # Little-endian
    #
    # e.g. for 0xDEAD_BEEF:
    # 1101 1110 1010 1101 1011 1110 1110 1111
    # 3333 3333 2222 2222 1111 1111 0000 0000
    # packed = [ 0xEF, 0xBE, 0xAD, 0xDE ]
    return bytearray([
    integer & 0xFF,       # bits [7, 0]
    integer >> 8 & 0xFF,  # bits [15, 8]
    integer >> 16 & 0xFF, # bits [23, 16]
    integer >> 24 & 0xFF  # bits [31, 24]
    ])

def pack_16(integer):
    # Little-endian
    #
    # e.g. for 0xBEEF:
    # 1011 1110 1110 1111
    # 1111 1111 0000 0000
    # packed = [ 0xEF, 0xBE ]
    return bytearray([
    integer & 0xFF,     # bits [7, 0]
    integer >> 8 & 0xFF # bits [15, 8]
    ])

def firmatify_32(pack):
    # Convert a packed bytearray to the 7-bit packets Firmata receives.
    # Useful for checking a == decode_32(firmatify_32(pack_32(a)))
    #
    # e.g.  for [0xEF, 0xBE, 0xAD, 0xDE]:
    # [1101 1110, 1010 1101, 1011 1110, 1110 1111]
    # firmatafied = [ 0101 1110, 0000 0001, 0010 1101, 0000 0001, 0011 1110, 0000 0001, 0110 1111, 0000 0001 ]
    #             = [ 0x5E, 0x01, 0x2D, 0x01, 0x3E, 0x01, 0x6F, 0x01]
    return bytearray([
    pack[0] & 0x7F,
    (pack[0] & 0x80) >> 7,
    pack[1] & 0x7F,
    (pack[1] & 0x80) >> 7,
    pack[2] & 0x7F,
    (pack[2] & 0x80) >> 7,
    pack[3] & 0x7F,
    (pack[3] & 0x80) >> 7,
    ])

def decode_32(data):
    # Decode Firmata 7-bit packets into a 32-bit integer
    # See firmatify_32 for an explanation of what Firmata does to packets
    return data[0] | data[1] << 7 \
        | (data[2] | data[3] << 7) << 8 \
        | (data[4] | data[5] << 7) << 16 \
        | (data[6] | data[7] << 7) << 24

def decode_16(data):
    # Decode Firmata 7-bit packets into a 16-bit integer
    # See firmatify_32 for an explanation of what Firmata does to packets
    return data[0] | data[1] << 7 \
        | (data[2] | data[3] << 7) << 8

def on_echo_text(*data):
    print(util.two_byte_iter_to_str(data))

def on_echo_int32(*data):
    print(decode_32(data))

def on_echo_int16(*data):
    print(decode_16(data))

def on_echo_raw(*data):
    print(data)

def test(*data):
    print("On 0x71")
    on_echo_text(data)

# Setup Firmata
b = Arduino(COM_PORT)
b.add_cmd_handler(0x71, test)

# Send text echo
print(b.bytes_available())
b.add_cmd_handler(0x00, on_echo_text)
b.send_sysex(SYSEX_COMMAND_ECHO, util.str_to_two_byte_iter("hello world"))
print(b.bytes_available())
b.iterate()

time.sleep(1)

# Send some numerical echos
b.add_cmd_handler(0x00, on_echo_int32)
b.send_sysex(SYSEX_COMMAND_ECHO, pack_32(0xDEAD_BEEF))
b.iterate()
b.send_sysex(SYSEX_COMMAND_ECHO, pack_32(1000))
b.iterate()
b.send_sysex(SYSEX_COMMAND_ECHO, pack_32(32767))
b.iterate()

time.sleep(1)

# Send the numerical echos raw
b.add_cmd_handler(0x00, on_echo_raw)
b.send_sysex(SYSEX_COMMAND_ECHO, pack_32(0xDEAD_BEEF))
b.iterate()
b.send_sysex(SYSEX_COMMAND_ECHO, pack_32(1000))
b.iterate()
b.send_sysex(SYSEX_COMMAND_ECHO, pack_32(32767))
b.iterate()

# Send speed of 2 RPM for 5 seconds, then 1 RPM in other direction for 5 seconds, then stop
b.send_sysex(SYSEX_COMMAND_SET_SPEED, pack_16(20))
if (b.bytes_available()): b.iterate()
time.sleep(5)
b.send_sysex(SYSEX_COMMAND_SET_SPEED, pack_16(-10))
if (b.bytes_available()): b.iterate()
time.sleep(5)
b.send_sysex(SYSEX_COMMAND_SET_SPEED, pack_16(0))
if (b.bytes_available()): b.iterate()

