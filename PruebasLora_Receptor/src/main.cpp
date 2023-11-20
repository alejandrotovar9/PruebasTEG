#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>


#define ss 5
#define rst 16
#define dio0 4
#define LED 2

void setup() {
   Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Receiver");
  
  LoRa.setPins(ss, rst, dio0);    //setup LoRa transceiver module
 
  while (!LoRa.begin(433E6))     //433E6 - Asia, 866E6 - Europe, 915E6 - North America
  {
    Serial.println(".");
    delay(500);
  }
  LoRa.setSyncWord(0xA5);
  Serial.println("LoRa Initializing OK!");
}

void loop() {
    int packetSize = LoRa.parsePacket();    // try to parse packet
  if (packetSize) 
  {
    
    digitalWrite(LED, HIGH);
    Serial.print("Received packet '");
 
    while (LoRa.available())              // read packet
    {
      String LoRaData = LoRa.readString();
      Serial.print(LoRaData); 
    }
    Serial.print("' with RSSI ");         // print RSSI of packet
    Serial.println(LoRa.packetRssi());
    delay(2000);
    digitalWrite(LED,LOW);
  }
  }