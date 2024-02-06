/*
  LoRa Duplex communication

  Sends a message every half second, and polls continually
  for new incoming messages. Implements a one-byte addressing scheme,
  with 0xFF as the broadcast address.

  Uses readString() from Stream class to read payload. The Stream class'
  timeout may affect other functuons, like the radio's callback. For an

  created 28 April 2017
  by Tom Igoe
*/


#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

const int csPin = 5;          // LoRa radio chip select
const int resetPin = 14;       // LoRa radio reset
const int irqPin = 2;         // change for your board; must be a hardware interrupt pin

// //Handle de tareas
TaskHandle_t xHandle_poll_packet;
TaskHandle_t xHandle_send_packet;
TaskHandle_t xHandle_timer;

float floatArray[] = {3.14, 1.23, 4.56, 2.34, 5.67};
byte* byteArray = reinterpret_cast<byte*>(floatArray);
size_t byteArraySize = sizeof(floatArray);

String outgoing;              // outgoing message

byte msgCount = 0;            // count of outgoing messages
//OJO UNO ES 0xFF y el otro 0xbb, para que tengan diferentes identificadores
byte localAddress = 0xBB;     // address of this device
byte destination = 0xFF;      // destination to send to
long lastSendTime = 0;        // last send time
int interval = 2000;          // interval between sends
int expected_length = 125;


int leer_datos(int packetSize){

  if (packetSize == 0) return 0;          // if there's no packet, return 0

  Serial.println("Hay mensaje");

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  if (incomingLength != expected_length) {   // check length for error
    Serial.println("error: message length does not match expected length");
    return 1;                             // skip rest of function
  }

  byte incoming[incomingLength]; //Crea un byte array del tamaño de los datos enviados
  	
  //Guardo datos en un vector para su posterior utilización
  //Existe manera mas rápida de guardarlos?
  int k = 0;
  while (LoRa.available()) {
    incoming[k] = (byte)LoRa.read();
    k++;
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me.");
    return 2;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  //Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println("El primer byte del mensaje es: " + String(incoming[0], HEX));
  Serial.println();
  return 3;
}

void sendMessage(size_t size_data, byte data[]) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(size_data);      // add payload length
  //LoRa.print(outgoing);                 // add payload
  LoRa.write(data, size_data);
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}

//Variables goblales para excepcion y conteo de errores
int error_count = 0;
int general_count = 0;

void poll_packet(void *pvParameters){
  while(1){
    //Serial.println("Polling...");
    int packetSize = LoRa.parsePacket();
    int flag = leer_datos(packetSize);

    //--------------------------Manejo de excepciones de receptor------------------------------
    switch(flag){
      case 0: break;//No se recibio un paquete, seguir haciendo polling
      case 1: 
        Serial.println("#####################################################");
        Serial.println("Los datos estan corruptos");
        Serial.println("#####################################################");
        error_count++;
        printf("Los errores hasta ahora son: %d/%d \n", error_count, general_count);
        break;//La data no es de la longitud deseada (Data corrupted)
      case 2:
        Serial.println("#####################################################");
        Serial.println("Los datos estan corruptos o no son para este receptor");
        Serial.println("#####################################################");
        break;
         //El mensaje no es para el
      case 3:
        Serial.println("Se recibio un paquete!!! Reactivando tarea de envio de mensaje...");
        general_count++;
        if(general_count == 255){
          Serial.println("#####################################################");
          Serial.println("#####################################################");
          Serial.println("#####################################################");
          printf("Los errores para 255 envios fueron: %d/%d \n", error_count, general_count);
        }
        vTaskResume(xHandle_send_packet); //La data llego con exito y tiene el formato deseado
        //Aqui se enviarian los datos para ser guardados y enviados por MQTT
    }
    // if(leer_datos(packetSize)){ //AQUI PUEDE IR EL CONTADOR DE IDs para enviar mensaje de verificacion
    //   //Se recibio algo
    //   Serial.println("Se recibio un paquete!!! Reactivando tarea de envio de mensaje...");
    //   vTaskResume(xHandle_send_packet);
    // }
    // else{
    //   //vTaskDelay(1000/portTICK_PERIOD_MS);
    // }
      // if(leer_datos(packetSize) == 1){
      //   Serial.println("Recepcion correcta");
      // }
      // else{
      //   Serial.println("Error en el envio...");
      //   vTaskDelay(500/portTICK_PERIOD_MS);
      // }

      //Serial.println("No han llegado datos...");
      vTaskDelay(10/portTICK_PERIOD_MS);

      //Activa la tarea de envio de datos una vez se recibe algo
      //vTaskResume(xHandle_send_packet);
    }
}

void send_packet_nollego(void *pvParameters){
  while(1){
    //Byte array a enviar
    size_t size_data;
    byte data[]={0x00}; 
    size_data = sizeof(data);

    //Llamo a la funcion que crea la trama de datos (payload)
    sendMessage(size_data, data);
    Serial.println("Sending byte array de respuesta negativa!");

    //Espero 2 segundos luego de enviar mensaje
    //vTaskDelay(2000/portTICK_PERIOD_MS);

    //vTaskResume(xHandle_poll_packet);

    //Suspendo esta tarea hasta que se reciba otro mensaje
    vTaskSuspend(NULL);
  }  
}

void send_packet(void *pvParameters){
  while(1){
    //Byte array a enviar
    size_t size_data;
    byte data[]={0x01}; 
    /*
    100 bytes

    0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02,0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05,
    0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01
    */

    size_data = sizeof(data);

    //Llamo a la funcion que crea la trama de datos (payload)
    sendMessage(size_data, data);
    Serial.println("Sending byte array de respuesta!");
    Serial.println();

    //Espero 2 segundos luego de enviar mensaje
    //vTaskDelay(2000/portTICK_PERIOD_MS);

    //Suspendo esta tarea hasta que se reciba otro mensaje
    vTaskSuspend(NULL);
  }  
}

void setup() {
  Serial.begin(115200);                   // initialize serial
  while (!Serial);

  Serial.println("LoRa Duplex");

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  if (!LoRa.begin(433E6)) {             // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  Serial.println("LoRa init succeeded.");

  //Tareas de FreeRTOS corriendo en el nucleo 0 del ESP32
  xTaskCreatePinnedToCore(send_packet, "send_packet", 1024*2, NULL, 1, &xHandle_send_packet, 0);
  vTaskSuspend(xHandle_send_packet);
  xTaskCreatePinnedToCore(poll_packet, "poll_packet", 1024*2, NULL, 1, &xHandle_poll_packet, 0);
  
  //xTaskCreatePinnedToCore(timer, "timer", 1024*2, NULL, 1, &xHandle_timer, 0);
}

void loop() {
  // Do nothing
}
