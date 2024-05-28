import pyfirmata2
import time

COM_PORT = 'COM3'
SYSEX_COMMAND_ECHO = 0x00

def pack_32(integer):
    # Little-endian
    #
    # e.g. for 0xDEAD_BEEF:
    # 1101 1110 1010 1101 1011 1110 1110 1111
    # 3333 3333 2222 2222 1111 1111 0000 0000
    # packed = [ 0xEF, 0xBE, 0xAD, 0xDE ]
    return bytearray([
    integer & 0xFF, # bits [7, 0]
    integer >> 8 & 0xFF, # bits [15, 8]
    integer >> 16 & 0xFF, # bits [23, 16]
    integer >> 24 & 0xFF]) # bits [31, 24]

# THIS DOES NOT DECODE pack_32 OUTPUT
# Firmata can only send 7 bit packets, so instead it sends two 7 bits that need to be reconstructed
# This decodes Firmata output back into a 32 bit block
def decode_32(data):
    return data[0] | data[1] << 7 | (data[2] | data[3] << 7) << 8 | (data[4] | data[5] << 7) << 16 | (data[6] | data[7] << 7) << 24

def rec_echo_text(*data):
    print(util.two_byte_iter_to_str(data))

def rec_echo_int32(*data):
    print(decode_32(data))

def rec_echo_raw(*data):
    print(data)

# Setup Firmata
b = Arduino(COM_PORT)
it = util.Iterator(b)
it.start()

# Send text echo
b.add_cmd_handler(0x00, rec_echo)
b.send_sysex(SYSEX_COMMAND_ECHO, util.str_to_two_byte_iter("hello world"))

time.sleep(1)

# Send some numerical echos
b.add_cmd_handler(0x00, rec_echo_int32)
b.send_sysex(SYSEX_COMMAND_ECHO, pack_32(0xDEAD_BEEF))
b.send_sysex(SYSEX_COMMAND_ECHO, pack_32(1000))
b.send_sysex(SYSEX_COMMAND_ECHO, pack_32(32767))

time.sleep(1)

# Send the numerical echos raw
b.add_cmd_handler(0x00, rec_echo_raw)
b.send_sysex(SYSEX_COMMAND_ECHO, pack_32(0xDEAD_BEEF))
b.send_sysex(SYSEX_COMMAND_ECHO, pack_32(1000))
b.send_sysex(SYSEX_COMMAND_ECHO, pack_32(32767))
