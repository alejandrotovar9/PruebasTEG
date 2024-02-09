/*
EMISOR DE DATOS LORA
Jose Tovar
06/02/2024
Trabajo Especial de Grado - EIE
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
TaskHandle_t xHandle_poll_modo_operacion;
TaskHandle_t xHandle_timer;

float floatArray[] = {3.14, 1.23, 4.56, 2.34, 5.67};
byte* byteArray = reinterpret_cast<byte*>(floatArray);
size_t byteArraySize = sizeof(floatArray);

String outgoing;              // outgoing message

byte msgCount = 0;            // count of outgoing messages
//OJO UNO ES 0xFF y el otro 0xbb, para que tengan diferentes identificadores
byte localAddress = 0xFF;     // address of this device
byte destination = 0xBB;      // destination to send to
long lastSendTime = 0;        // last send time
int interval = 2000;          // interval between sends

int NUM_PAQUETES_ESPERADOS = 32; //Numero de paquetes que se esperan enviar
int expected_length = 1; //Longitud esperada de los datos enviados por el receptor

int leer_datos_sensor(int packetSize){

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
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println("El byte del mensaje es: " + String(incoming[0], HEX));
  Serial.println();

  if (incoming[0] = 0x01){
    //Se recibio un paquete pero aun faltan
    return 3;
  }
  else if(incoming[0] = 0x02){
    //Se recibieron todos los paquetes en el receptor, no enviar mas paquetes
    return 4;
  }
  else{
    //El paquete no se recibio con exito
    return 5;
  }
}

int leer_modo_op(int packetSize){

  if (packetSize == 0) return 0;          // if there's no packet, return 0

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  if (incomingLength != expected_length) {   // check length for error
    Serial.println("error: message length does not match expected length");
    return 5;                             // skip rest of function
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
    return 6;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println("El modo de operacion es: " + String(incoming[0], HEX));
  Serial.println();

  if (incoming[0] = 0x01){
    //Se selecciono el modo de operacion 1
    return 1;
  }
  else if(incoming[0] = 0x02){
    //Se selecciono el modo de operacion 2
    return 2;
  }
  else{
    //No se recibio un modo de operacion o se selecciono el modo 3
    return 5;
  }
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

//RUTINA DE POLLING PARA VERIFICAR EL MODO DE OPERACION DEL SENSORE INTELIGENTE
void poll_modo_operacion(void *pvParameters){
  while(1){
    int packetSize = LoRa.parsePacket();
    int modo_de_operacion = leer_modo_op(packetSize);

    switch(modo_de_operacion){
      case 0: break; //No se recibio ningun mensaje, seguir haciendo polling

      case 1: //Modo de operacion 1: Envio de trama de datos
      //Rutinas del modo de operacion 1
      Serial.println("Se selecciono el modo de operacion 1: Envio de trama de datos inmediata");
      vTaskResume(xHandle_send_packet); //Rutina de envio de trama de datos
      vTaskResume(xHandle_poll_packet); //Rutina de polling para mensajes del receptor'
      vTaskSuspend(NULL); //Suspendo temporalmente el polling de modo de operacion
      break;

      case 2: //Modo de operacion 2: Envio de datos continuos
      //Rutinas del modo de operacion 2
      Serial.println("Se selecciono el modo de operacion 2: Envio de datos continuos");
      break;

      case 5: //Datos corruptos
      Serial.println("Los datos estan corruptos");
      break;

      case 6: //Mensaje no es para mi
      Serial.println("Los datos no son para mi");
      break;
    }

    vTaskDelay(10/portTICK_PERIOD_MS);
  }
}

//Variables goblales para excepcion y conteo de errores
int error_count = 0;
int general_count = 0;

//RUTINA DE POLLING PARA RECIBIR DATOS DEL RECEPTOR CUANDO ESTA RECIBIENDO LA DATA DE LOS SENSORES
void poll_packet(void *pvParameters){
  while(1){
    //Serial.println("Polling...");
    int packetSize = LoRa.parsePacket();
    int flag = leer_datos_sensor(packetSize);

    //--------------------------Manejo de excepciones de receptor------------------------------
    switch(flag){
      case 0: break;//No se recibio un paquete, seguir haciendo polling
      case 1:
        general_count++; 
        Serial.println("#####################################################");
        Serial.println("Los datos estan corruptos");
        Serial.println("#####################################################");
        error_count++;
        printf("Los errores hasta ahora son: %d/%d \n", error_count, general_count);
        break;//La data no es de la longitud deseada (Data corrupted)
      case 2:
        general_count++;
        Serial.println("#####################################################");
        Serial.println("Los datos no son para este receptor");
        Serial.println("#####################################################");
        break;
         //El mensaje no es para el
      case 3:
        Serial.println("Se recibio un paquete desde el receptor!!!");
        general_count++;
        Serial.println("#####################################################");
        printf("Los errores para %d envios fueron: %d/%d \n", general_count, error_count, general_count);
        //vTaskResume(xHandle_send_packet); //La data llego con exito y tiene el formato deseado
      case 4:  //Respondo al emisor con que los datos ya fueron enviados por completo
        Serial.println("Se recibieron todos los paquetes desde el receptor!!!");
        Serial.println("Desactivando rutina de envio de datos en el emisor...");
        vTaskSuspend(xHandle_send_packet);
        Serial.println("Reactivando rutina de lectura de modo de operacion del sensor...");
        vTaskResume(xHandle_poll_modo_operacion);
        Serial.println("Desactivando esta rutina de polling recepcion de datos de sensores");
        vTaskSuspend(NULL);
      case 5:
        Serial.println("Se recibio un paquete en el receptor pero no se reconoce el mensaje");
        break;
    }

      //Serial.println("No han llegado datos...");
      vTaskDelay(10/portTICK_PERIOD_MS);

      //Activa la tarea de envio de datos una vez se recibe algo
      //vTaskResume(xHandle_send_packet);
    }
}

//RUTINA DE ENVIO DE DATOS DE SENSORES A ESTACION BASE
void send_packet(void *pvParameters){
  while(1){
    //Byte array a enviar

    //Debo recibir los datos a enviar en una cola y formatearlos de ser necesario

    size_t size_data;
    byte data[]={0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02,0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05,
    0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01,
    0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02,0x01, 0x05, 0x04, 0x01, 0x05, 0x04, 0x03, 0x02,0x01, 0x05, 0x04}; 

    size_data = sizeof(data);

    //Llamo a la funcion que crea la trama de datos (payload)
    sendMessage(size_data, data);
    Serial.println("Sending byte array con datos!");
    Serial.println();

    //Espero X segundos luego de enviar mensaje
    vTaskDelay(500/portTICK_PERIOD_MS);

    //Suspendo esta tarea hasta que se reciba otro mensaje
    //vTaskSuspend(NULL);
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

  LoRa.setSignalBandwidth(125E3);//7.8E3 hasta 250E3, por defecto es 31.25E3
  LoRa.setSpreadingFactor(7);//entre 6 y 12
  //LoRa.enableCrc();

  Serial.println("LoRa init succeeded.");

  //Tareas de FreeRTOS corriendo en el nucleo 0 del ESP32
  xTaskCreatePinnedToCore(send_packet, "send_packet", 1024*2, NULL, 1, &xHandle_send_packet, 0);
  vTaskSuspend(xHandle_send_packet);
  xTaskCreatePinnedToCore(poll_packet, "poll_packet", 1024*2, NULL, 1, &xHandle_poll_packet, 0);
  vTaskSuspend(xHandle_poll_packet);
  xTaskCreatePinnedToCore(poll_modo_operacion, "poll_modo_op", 1024*2, NULL, 1, &xHandle_poll_modo_operacion, 0);
  
}

void loop() {
  // Do nothing
}

// //Tarea de FreeRTOS encargada de generar una ISR si recibo un mensaje por LoRa
// void IRAM_ATTR onReceive(int packetSize) {
//   //Serial.println("Recibido mensaje");
//   //vTaskResume(xHandle_poll_packet);
// }

// //Tarea para crear una ISR en el ESP32 si presiono un boton
// void IRAM_ATTR onButtonPress() {
//   //Serial.println("Boton presionado");
//   //vTaskResume(xHandle_send_packet);
// }