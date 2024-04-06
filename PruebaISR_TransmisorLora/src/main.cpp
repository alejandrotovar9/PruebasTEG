#include <Arduino.h>
#include <Wire.h>

/*
  RadioLib SX127x Blocking Transmit Example

  This example transmits packets using SX1278 LoRa radio module.
  Each packet contains up to 255 bytes of data, in the form of:
  - Arduino String
  - null-terminated char array (C-string)
  - arbitrary binary data (byte array)

  Other modules from SX127x/RFM9x family can also be used.

  Using blocking transmit is not recommended, as it will lead
  to inefficient use of processor time!
  Instead, interrupt transmit is recommended.

  For default module settings, see the wiki page
  https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem

  For full API reference, see the GitHub Pages
  https://jgromes.github.io/RadioLib/
*/

// include the library
#include <RadioLib.h>

struct Packet {
  byte messageId;
  byte senderId;
  byte receiverId;
  byte payload[128]; // Adjust the size as needed
};

// SX1278 has the following connections:
// NSS pin:   10
// DIO0 pin:  2
// RESET pin: 9
// DIO1 pin:  3
SX1278 radio = new Module(5, 2, 14, 3);

// or using RadioShield
// https://github.com/jgromes/RadioShield
//SX1278 radio = RadioShield.ModuleA;

byte byteArr[128];

void setup() {
  Serial.begin(115200);

  // initialize SX1278 with default settings
  Serial.print(F("[SX1278] Initializing ... "));
  int state = radio.begin();
  radio.setSpreadingFactor(7);
  radio.setBandwidth(250.0);
  // radio.setOutputPower(15);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  // some modules have an external RF switch
  // controlled via two pins (RX enable, TX enable)
  // to enable automatic control of the switch,
  // call the following method
  // RX enable:   4
  // TX enable:   5
  /*
    radio.setRfSwitchPins(4, 5);
  */
  for (int i = 0; i < 128; i++) {
    byteArr[i] = i;
  }
}

// counter to keep track of transmitted packets
int count = 0;

void loop() {
  Serial.print(F("[SX1278] Transmitting packet ... "));

//byte byteArr[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};
    // int state = radio.transmit(byteArr, 8);

Packet packet;

packet.messageId = count++;
packet.senderId = 1; // Set your sender ID
packet.receiverId = 2; // Set the intended receiver ID
memcpy(packet.payload, byteArr, sizeof(packet.payload)); // Copy your byte array into the payload

// Convert the Packet struct to a byte array
byte* packetBytes = reinterpret_cast<byte*>(&packet);

int state = radio.startTransmit(packetBytes, sizeof(packet));

// Create a byte array to hold the packet
// byte packet[15] = {0}; // Adjust the size as needed

// // Set the messageId, senderId, and receiverId
// packet[0] = count++; // messageId
// packet[1] = 1; // senderId
// packet[2] = 2; // receiverId

// // Set the payload
// byte payload[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};
// memcpy(&packet[3], payload, sizeof(payload));

// // Now you can transmit the packet
// int state = radio.startTransmit(packet, sizeof(packet));

  // you can transmit C-string or Arduino string up to
  // 255 characters long
  //  String str = "Hello World! #" + String(count++);
  //  int state = radio.startTransmit(str);

  // you can also transmit byte array up to 256 bytes long
     //byte byteArr[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};
    // int state = radio.transmit(byteArr, 8);

  //int state = radio.startTransmit(byteArr, 8);


  if (state == RADIOLIB_ERR_NONE) {
    // the packet was successfully transmitted
    Serial.println(F(" success!"));

    // print measured data rate
    Serial.print(F("[SX1278] Datarate:\t"));
    Serial.print(radio.getDataRate());
    Serial.println(F(" bps"));

  } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 256 bytes
    Serial.println(F("too long!"));

  } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
    // timeout occurred while transmitting packet
    Serial.println(F("timeout!"));

  } else {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);
  }

  // wait for a second before transmitting again
  delay(500);
}