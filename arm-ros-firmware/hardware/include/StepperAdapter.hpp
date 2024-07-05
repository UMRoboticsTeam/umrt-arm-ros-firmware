#ifndef SPI_COMMS_HPP_
#define SPI_COMMS_HPP_

#include <vector>

class StepperAdapter {
public:
    void init();

    void connect();

    void disconnect();

    void destroy();

    std::vector<uint32_t> readPosition();

    void setValues();
};


#endif