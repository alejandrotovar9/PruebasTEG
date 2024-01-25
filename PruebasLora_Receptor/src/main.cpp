#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>


#define ss 5
#define rst 14
#define dio0 2
#define LED 4

const int maxPayloadSize = 255;

// //Handle de tareas
TaskHandle_t xHandle_lora_receive_packet;
TaskHandle_t xHandle_lora_send_packet;

void lora_receive_packet(void *pvParameters){
   // Check if there is any incoming data
  while(1)
  {
    //Serial.println("Esperando datos...");
    int packetSize;
    packetSize = LoRa.parsePacket();
    if (packetSize > 0) {
      Serial.print("Received packet from ESP32 2 (Receptor/E.B): ");
      //Read data and print it
      char receivedData[maxPayloadSize];
      size_t dataSize = LoRa.readBytes(receivedData, maxPayloadSize);

      Serial.println(receivedData);

      //Se envia siguiente mensaje
      // vTaskResume(&xHandle_lora_send_packet);
      // vTaskSuspend(&xHandle_lora_receptor);
    }
    vTaskDelay(10/portTICK_PERIOD_MS);

    // else{
    //   //Do nothing
    // }
  }

  //  // Check if there is any incoming data
  // while(1)
  // { 
  //   String LoRaData;
  //   // LoRa data packet size received from LoRa sender
  //   int packetSize = LoRa.parsePacket();
  //   // if the packer size is not 0, then execute this if condition
  //   if (packetSize) {
  //     // received a packet
  //     Serial.print("Received packet: ");

  //     // receiving the data from LoRa sender
  //     while (LoRa.available()) {
  //       LoRaData = LoRa.readString();
  //     }
  //     Serial.println(LoRaData);
  //   }
  // }
}

void lora_send_packet(void *Parameters){
  while(1)
     {  
      Serial.println("Enviando paquete desde estacion base...");
      //Serial.println(counter);
    
      //byte data[5] = {0x01, 0x02, 0x03, 0x04, 0x05};
      //data[0] = counter; 

      LoRa.beginPacket();   //Send LoRa packet to receiver
      LoRa.print("Hola ESP32 1!!!");
      //LoRa.write(data, sizeof(data));
      //LoRa.print(counter);
      LoRa.endPacket();

      //counter++;

      // vTaskResume(&xHandle_lora_receptor);
      // vTaskSuspend(&xHandle_lora_send_packet);

      int num = random(100) + 3000;
    
      vTaskDelay(num/portTICK_PERIOD_MS);
    }
}

void lora_setup(void){
  LoRa.setPins(ss, rst, dio0);    //setup LoRa transceiver module
 
  while (!LoRa.begin(433E6))     //433E6 - Asia, 866E6 - Europe, 915E6 - North America
  {
    Serial.println(".");
    delay(500);
  }
  LoRa.setSpreadingFactor(6); //Valor entre 6 y 12
  LoRa.setSyncWord(0xA5);
  Serial.println("LoRa Initializing OK!");
}

void setup() {
   Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Receiver");
  lora_setup();

  xTaskCreatePinnedToCore(lora_receive_packet, "lora_receive_packet", 1024*4, NULL, 1, &xHandle_lora_receive_packet, 0);  
  //xTaskCreatePinnedToCore(lora_send_packet, "lora_send_packet", 1024*4, NULL, 1, &xHandle_lora_send_packet, 1);
  // vTaskSuspend(&xHandle_lora_send_packet);  

}

void loop() {
    //do nothing
  }
