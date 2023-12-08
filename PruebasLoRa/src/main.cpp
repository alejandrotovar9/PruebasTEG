#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

int counter = 0;

#define ss 4
#define rst 5
#define dio0 2

void setup() {
   Serial.begin(115200); 
  while (!Serial);
  Serial.println("LoRa Sender");
 
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
 Serial.print("Sending packet: ");
  Serial.println(counter);
 
  LoRa.beginPacket();   //Send LoRa packet to receiver
  LoRa.print("hello ");
  LoRa.print(counter);
  LoRa.endPacket();
 
  counter++;
 
  delay(5000);
}