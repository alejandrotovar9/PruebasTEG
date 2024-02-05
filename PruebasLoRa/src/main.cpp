#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

const int maxPayloadSize = 255;
int counter = 0;

#define ss 5
#define rst 14
#define dio0 2

// //Handle de la Cola
// QueueHandle_t xQueue;
// QueueHandle_t dataQueue;

//float prueba[10];

float floatArray[] = {3.14, 1.23, 4.56, 2.34, 5.67};
byte* byteArray = reinterpret_cast<byte*>(floatArray);
size_t byteArraySize = sizeof(floatArray);

// //Handle de tareas
// TaskHandle_t xHandle_mqttTask;
TaskHandle_t xHandle_lora_send_packet;
TaskHandle_t xHandle_lora_receptor;


void lora_setup(void){
  //La libreria crea un objeto LoRa de la clase LoRaClass
  LoRa.setSpreadingFactor(6); //valor entre 6 y 12
  LoRa.setPins(ss, rst, dio0);    //setup LoRa transceiver module
  
  while (!LoRa.begin(433E6))     //433E6 - Asia, 866E6 - Europe, 915E6 - North America
  {
    Serial.println(".");
    delay(500);
  }
  LoRa.setSyncWord(0xA5);
  Serial.println("LoRa Initializing OK!");
}

void lora_receptor(void *pvParameters){
   // Check if there is any incoming data
  while(1)
  { 
    String LoRaData;
    // LoRa data packet size received from LoRa sender
    int packetSize = LoRa.parsePacket();
    // if the packer size is not 0, then execute this if condition
    //Serial.println("Esperando datos...");
    if (packetSize) {
      // received a packet
      Serial.print("Received packet: ");

      // receiving the data from LoRa sender
      while (LoRa.available()) {
        LoRaData = LoRa.readString();
      }
      Serial.println(LoRaData);
    }
    vTaskDelay(200/portTICK_PERIOD_MS);
  }

  //  // Check if there is any incoming data
  // while(1)
  // {
  //   Serial.println("Esperando datos...");
  //   if (LoRa.parsePacket()) {
  //     Serial.print("Received packet from ESP32 2 (Receptor/E.B): ");
  //     //Read data and print it
  //     char receivedData[maxPayloadSize];
  //     size_t dataSize = LoRa.readBytes(receivedData, maxPayloadSize);

  //     Serial.println(receivedData);

  //     //Se envia siguiente mensaje
  //     // vTaskResume(&xHandle_lora_send_packet);
  //     // vTaskSuspend(&xHandle_lora_receptor);
  //   }
  //   vTaskDelay(500/portTICK_PERIOD_MS);

  //   // else{
  //   //   //Do nothing
  //   // }
  // }
}

void lora_send_packet(void *Parameters){
  while(1)
    {  
      Serial.print("Sending packet: ");
      Serial.println(counter);
    
      //byte data[10] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10};
      //data[0] = counter; 

      LoRa.beginPacket();   //Send LoRa packet to receiver
      LoRa.print("Hola ");
      //LoRa.write(data, sizeof(data));
      //LoRa.print(counter);
      LoRa.endPacket();

      counter++;

      // vTaskResume(&xHandle_lora_receptor);
      // vTaskSuspend(&xHandle_lora_send_packet);
      int num = random(100) + 2000;

      vTaskDelay(num/portTICK_PERIOD_MS);
    }
}

void loop() {
 //Do nothing...
}

void setup() {
   Serial.begin(115200); 
  while (!Serial);

  Serial.println("LoRa Sender");

  lora_setup();

  //Tareas a ejecutarse en el Nucleo 1 (lectura de hum y temp)
  // xTaskCreatePinnedToCore(readBMETask, "readBMETask", 1024*2, NULL, 1, &xHandle_readBMETask, 1);
  xTaskCreatePinnedToCore(lora_send_packet, "lora_send_packet", 1024*1, NULL, 1, &xHandle_lora_send_packet, 1);
  //xTaskCreatePinnedToCore(lora_receptor, "lora_receptor", 1024*1, NULL, 1, &xHandle_lora_receptor, 0);
  // vTaskSuspend(&xHandle_lora_receptor);
}
