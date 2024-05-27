/* To Communicate (python commands):
 * from pySerialTransfer import pySerialTransfer as txfer
 * link = txfer.SerialTransfer(<INSERT COM PORT HERE>)
 * link.open()
 * 
 */ 


#include "ConnectionManager.hpp"

// MobaTools
// https://github.com/MicroBahner/MobaTools/tree/master
// For convenient driving stepper motors
#include <MobaTools.h>

// SerialTransfer
// https://github.com/PowerBroker2/SerialTransfer/tree/master
// For communication between USB master and Arduino
#include <SerialTransfer.h>

#define SERIAL_SPEED 115200

#define BUFF_SIZE 256

#define STEP_1 3
#define DIR_1 2
#define CS_1 4
#define CURRENT_1 2000
#define FULLROT_1 200

ConnectionManager manager;
SerialTransfer usb_transfer;
MoToStepper step_1(FULLROT_1, STEPDIR);

void setup() {
  // put your setup code here, to run once:
  ConnectionManager::init();

  // Setup SerialTransfer
  Serial.begin(SERIAL_SPEED);
  usb_transfer.begin(Serial);
  

  manager.add_driver(STEP_1, DIR_1, CS_1, CURRENT_1);

  step_1.attach(STEP_1, DIR_1);

  // =========TESTING=========
  step_1.setSpeed(10000); // 1000 RPM
  step_1.rotate(1);
  // =========================
}

void loop() {
  uint16_t send_size = 0;
  uint16_t rec_size = 0;
  char ack_prefix[] = "ack{";
  char ack_suffix[] = "}\n";
  char tx_buff[BUFF_SIZE + sizeof(ack_prefix) + sizeof(ack_suffix) - 2 + 1]; // -2 because ack_prefix and ack_suffix both have \0s + 1 because we want a \0
  char rx_buff[BUFF_SIZE + 1]; // +1 so we have space for a \0
  
  // Uncomment to endlessly scream the ack_prefix
  //send_size = usb_transfer.txObj(ack_prefix, send_size);
  //usb_transfer.sendData(send_size);
  

  // Wait until we can receive
  if (usb_transfer.available()){
    // Receive the string and terminate it
    rec_size = usb_transfer.rxObj(rx_buff, 0, BUFF_SIZE);
    rx_buff[rec_size] = '\0';

    // Construct the response and send it
    //snprintf(tx_buff, sizeof(tx_buff), "%s%s%s", ack_prefix, rx_buff, ack_suffix);
    snprintf(tx_buff, sizeof(tx_buff), "%s%d,%s%s", ack_prefix, rec_size, rx_buff, ack_suffix);
    send_size = usb_transfer.txObj(tx_buff, send_size);
    usb_transfer.sendData(send_size);
  }
  else if (usb_transfer.status < 0){
    // Uh oh... not sure what we can do
  }
}
