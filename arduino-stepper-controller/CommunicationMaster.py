import pyfirmata2 as pyfirmata2

COM_PORT = 'COM3'
SYSEX_COMMAND_ECHO = 0x00

def pack_32(integer):
    # Little-endian in 7 bit blocks:
    # bbbb bbbb bbbb bbbb bbbb bbbb bbbb bbbb
    # 4444 3333 3332 2222 2211 1111 1000 0000
    # 
    # e.g. for 0xDEAD_BEEF:
    # 1101 1110 1010 1101 1011 1110 1110 1111
    # 4444 3333 3332 2222 2211 1111 1000 0000
    # packed = [ 0x6F, 0x7D, 0x36, 0x75, 0x0D ]
    return bytearray([
    integer & 0x7F, # bits [6, 0]
    integer >> 7 & 0x7F, # bits [13, 7]
    integer >> 14 & 0x7F, # bits [20, 14]
    integer >> 21 & 0x7F, # bits [27, 21]
    integer >> 28 & 0x0F]) # bits [31, 28]

def unpack_32(data):
    return (data[0] & 0x7F) | (data[1] & 0x7F) << 7 | (data[2] & 0x7F) << 14 | (data[3] & 0x7F) << 21 | (data[4] & 0x0F) << 28

def rec_echo_text(*data):
    print(util.two_byte_iter_to_str(data))

def rec_echo_int32(*data):
    print(unpack_32(data)) # Need to only look at every other byte

def rec_echo_raw(*data):
    print(data) # Arduino returns 2 bytes

# Setup Firmata
b = Arduino(COM_PORT)
b.add_cmd_handler(0x00, rec_echo)
it = util.Iterator(b)
it.start()

# Send text echo
b.send_sysex(SYSEX_COMMAND_ECHO, util.str_to_two_byte_iter("hello world"))

# Send some numerical echos
b.add_cmd_handler(0x00, rec_echo_int32)
b.send_sysex(SYSEX_COMMAND_ECHO, pack_32(0xDEAD_BEEF))
b.send_sysex(SYSEX_COMMAND_ECHO, pack_32(1000))
b.send_sysex(SYSEX_COMMAND_ECHO, pack_32(32767))

# Send the numerical echos raw
b.add_cmd_handler(0x00, rec_echo_raw)
b.send_sysex(SYSEX_COMMAND_ECHO, pack_32(0xDEAD_BEEF))
b.send_sysex(SYSEX_COMMAND_ECHO, pack_32(1000))
b.send_sysex(SYSEX_COMMAND_ECHO, pack_32(32767))
