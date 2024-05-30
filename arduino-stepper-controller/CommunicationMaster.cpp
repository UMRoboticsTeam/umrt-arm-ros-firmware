#include <iostream>
#include <string>
#include "firmata/serial_port.hpp"
#include "firmata/client.hpp"

using namespace firmata::literals;

const std::string COM_PORT = "COM3";
const baud_rate BAUD_RATE = 57600_baud;
const firmata::msg_id SYSEX_ECHO = (firmata::msg_id)0x00;

void read(firmata::msg_id msg, firmata::payload payload);
void rec_echo(firmata::payload payload);

int main(void)
{
    asio::io_service io;

    firmata::serial_port device(io, COM_PORT);
    device.set(BAUD_RATE);

    firmata::client arduino(device);

    device.write(SYSEX_ECHO, firmata::payload({0x6E}));
    device.on_read(read);

    asio::io_service::work work(io);
    io.run();

    // Run forever
    for(;;);

    std::cout << "Test" << std::endl;
}

void read(firmata::msg_id msg, firmata::payload payload){
    switch(msg){
        case SYSEX_ECHO: rec_echo(payload); break;
    }
}

void rec_echo(firmata::payload payload){
    std::cout<<payload[0]<<std::endl;
}