/*
   See documentation at https://nRF24.github.io/RF24
   See License information at root directory of this library
   Author: Brendan Doherty (2bndy5) test
*/

#include <SPI.h>
#include "printf.h"
#include "RF24.h"

// instantiate an object for the nRF24L01 transceiver
RF24 radio(7, 8); // using pin 7 for the CE pin, and pin 8 for the CSN pin

uint8_t address[][6] = {"1Node", "2Node"}; // Let these addresses be used for the pair

String temp_data = "";
char data[33] = {};
uint8_t bytes = 32;

void transmitString(String s) {
  int len = s.length() + 1;
  char data[len];
  s.toCharArray(data, len);
  radio.write(&data, sizeof(data));
}

void setup() {
  bool yes = false;
  Serial.begin(19200);
  // initialize the transceiver on the SPI bus
  while (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
  }

  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.

  // save on transmission time by setting the radio to only transmit the
  // number of bytes we need to transmit a float
  radio.openWritingPipe(address[0]); // set the TX address of the RX node into the TX pipe. Uses the address "1Node" to write. write always uses pipe 0.
  radio.openReadingPipe(1, address[1]); //set the RX address of the TX node into a RX pipe. Using pipe 1, use the address "2Node" to read.
  radio.stopListening();  // put radio in TX mode
  radio.setRetries(5, 15);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();


  // For debugging info
  printf_begin();             // needed only once for printing details
  // radio.printDetails();       // (smaller) function that prints raw register values
  //radio.printPrettyDetails(); // (larger) function that prints human readable data

  Serial.println("Send any character to initialize MPU6050 gyroscope/accelerometer");
  while (!yes) {
    if (Serial.available()) {
      temp_data = Serial.readString();
      Serial.print("Serial read: "); Serial.print(temp_data);
      temp_data.toCharArray(data, temp_data.length() + 1);
      if (radio.write(&data, sizeof(data))) {
        radio.startListening();
        Serial.println("listening");
        yes = true;
      } else {
        Serial.println("Initialization message failed to send!");
      }
    }
  }
}

void loop() {
  if (radio.available()) {
    bytes = radio.getDynamicPayloadSize();
    radio.read(&data, bytes);
    if (data[0] == '$') {
      Serial.println("drone initialized");
      radio.stopListening();
    } else if (data[0] == '*') {
      Serial.println("message recieved");
    }
  }

  if (Serial.available()) {
    temp_data = Serial.readString();
    temp_data.toCharArray(data, temp_data.length() + 1);
    radio.write(&data, sizeof(data));
  }
}
