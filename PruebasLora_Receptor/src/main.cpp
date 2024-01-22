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

const int maxPayloadSize = 10;

// //Handle de tareas
TaskHandle_t xHandle_lora_receive_packet;
TaskHandle_t xHandle_lora_send_packet;

void lora_receive_packet(void *pvParameters){
   // Check if there is any incoming data
  while(1)
  { 
    if (LoRa.parsePacket()) {
      Serial.print("Received packet: ");
      //Read data and print it
      byte receivedData[maxPayloadSize];
      size_t dataSize = LoRa.readBytes(receivedData, maxPayloadSize);

      for (size_t i = 0; i < dataSize; i++) {
        Serial.print(receivedData[i]);
        Serial.print(" ");
      }
      Serial.println();

      //Se envia mensaje de confirmacion
      //vTaskResume(&xHandle_lora_send_packet);
    }
  }  
  vTaskDelay(100/portTICK_PERIOD_MS); // Adjust the delay based on your requirements
}

void lora_send_packet(void *Parameters){
  while(1)
    {  
      Serial.print("Sending confirmation...");

      LoRa.beginPacket();   //Send LoRa packet to receiver
      LoRa.print("Si");
      LoRa.endPacket();
      vTaskSuspend(&xHandle_lora_send_packet);
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

  xTaskCreatePinnedToCore(lora_receive_packet, "lora_receive_packet", 1024*2, NULL, 1, &xHandle_lora_receive_packet, 1);  
  // xTaskCreatePinnedToCore(lora_send_packet, "lora_send_packet", 1024*2, NULL, 1, &xHandle_lora_send_packet, 0);
  // vTaskSuspend(&xHandle_lora_send_packet);  

}

void loop() {
    //do nothing
  }
