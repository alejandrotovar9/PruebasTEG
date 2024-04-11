#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <RadioLib.h>


#define SIZE_OF_FLOAT_ARRAY 1024
#define CHUNK_SIZE 32

#define NUM_DATOS 1024
#define MEAN_INTERVAL 5
#define NUM_DATOS_TEMP 200
#define NUM_DATOS_INC 800

//Estructuras de datos
struct BufferTempHumedad{
  float buffertemp[NUM_DATOS_TEMP/5 - 1] = { };
  float bufferhum[NUM_DATOS_TEMP/5 - 1] = { };
};

//Estructura que contiene 3 arreglos de 5000 flotantes
struct BufferACL{
  long int buffer_timestamp[NUM_DATOS - 1] = {};
  float bufferX[NUM_DATOS] = { };
  float bufferY[NUM_DATOS] = { };
  float bufferZ[NUM_DATOS] = { };
};

BufferACL buffer_prueba;

struct BufferInclinacion{
  float bufferRoll[NUM_DATOS_INC/5 - 1] = { };
  float bufferPitch[NUM_DATOS_INC/5 - 1] = { };
  float bufferYaw[NUM_DATOS_INC/5 - 1] = { };
};

struct Packet{
  byte messageID;
  byte senderID;
  byte receiverID;
  byte payload[128];
};

struct Packet2{
  byte messageId;
  byte senderId;
  byte receiverId;
  byte payload[1]; // Adjust the size as needed
};

// //Handle de tareas
TaskHandle_t xHandle_send_task;

TaskHandle_t xHandle_blink;

// //Handle de tareas
TaskHandle_t xHandle_receive_task = NULL;

//OJO UNO ES 0xFF y el otro 0xbb, para que tengan diferentes identificadores
byte localAddress = 0xBB;     // address of this device
byte destination = 0xFF;      // destination to send to

int NUM_PAQUETES_ESPERADOS = 96;
int expected_length = 128;

//Variables goblales para excepcion y conteo de errores
int error_count = 0;
int general_count = 0;

//Flag para controlar envio de datos
int flag_envio = 1;

//Prueba modo de op
int modo_de_operacion = 1;

//Contadores para funcion de recepcion de datos
int contador_errores =0;
long int contador = 0;

// counter to keep track of transmitted packets
int count = 0;

byte byteArr[1];


// SX1278 has the following connections:
// NSS pin:   10
// DIO0 pin:  2
// RESET pin: 9
// DIO1 pin:  3
SX1278 radio = new Module(5, 2, 14, 3);

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// flag to indicate transmission or reception state
bool transmitFlag = false;

void IRAM_ATTR ISR_button(){
  transmitFlag = true;
  vTaskResume(xHandle_send_task);
}


#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void ICACHE_RAM_ATTR setFlag(void) {
  // we got a packet, set the flag
  vTaskResume(xHandle_receive_task);
  receivedFlag = true;
}


void fillBufferX(float* buffer, unsigned char* byteArray, int size) {
    static int currentPos = 0;// keep track of current position in bufferX as static to prevent changes from function calls

    Serial.print("El valor actual de currentPos en la funcion fillBuffer> ");
    Serial.println(currentPos);

    for (int i = 0; i < size; i += 4) {
        // convert 4 bytes to float

        //creando la union
        union {
            float f;
            unsigned char b[4];
        } u;

        //Copiando 4 bytes del bytearray para convertir a flotante
        for (int j = 0; j < 4; j++) {
            u.b[j] = byteArray[i + j];
        }

        // append float to bufferX and increase position
        //bufferACL.bufferX[currentPos++] = u.f;
       
        // append float to buffer and increase position
        buffer[currentPos] = u.f;

        // if bufferX is full, reset currentPos to 0
        if (currentPos == NUM_DATOS - 1) {
          Serial.println("Reinicie currentPos");
          currentPos = 0;
        }
        else{
          currentPos++;
        }
    }
}

//Funcion para leer y verificar datos recibidos luego del polling
int leer_datos(size_t packetSize, byte incomingMsgId, byte sender, byte recipient, byte* incoming){

  if (packetSize == 0) return 0;          // if there's no packet, return 0

  if (packetSize != expected_length) {   // check length for error
    Serial.println("error: message length does not match expected length");
    return 1;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xBB) {
    Serial.println("Este mensaje no es para mi.");
    return 2;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  // Serial.println("Received from: 0x" + String(sender, HEX));
  // Serial.println("Sent to: 0x" + String(recipient, HEX));
  // Serial.println("Message ID: " + String(incomingMsgId));
  // Serial.println("Message length: " + String(packetSize));

    //Guardar incoming en buffer
  if((int)incomingMsgId <= 31){
    Serial.println("Guardando en bufferX!!!");
    //currentPos = 0;
      fillBufferX(buffer_prueba.bufferX, incoming, packetSize);  
  }
  else if((int)incomingMsgId > 31  && (int)incomingMsgId <= 63){
    Serial.println("Guardando en bufferY!!!");
    //currentPos = 0;
      fillBufferX(buffer_prueba.bufferY, incoming, packetSize);  
  }
  else if((int)incomingMsgId > 63 && (int)incomingMsgId <= 95){
    Serial.println("Guardando en bufferZ!!!");
    //currentPos = 0;
      fillBufferX(buffer_prueba.bufferZ, incoming, packetSize);  
  }

  // Serial.print("Se recibio el siguiente chunk: ");
  // //CHUNK_SIZE (cantidad de floats) * 4 (tamaño de un float) = 128 floats
  // for(int w= 0; w < CHUNK_SIZE * 4 ; w += sizeof(float)){
  //   float value;
  //   memcpy(&value, &incoming[w], sizeof(float));
  //   Serial.print(value);
  //   Serial.print(" ");
  // }

  //printf("El valor actual de msgID en int es: %d \n", int(incomingMsgId));

  if (int(incomingMsgId) < NUM_PAQUETES_ESPERADOS - 1) //Va del 0 al 31
  {
    return 3;
  }
  else{
    //Se recibieron todos los paquetes
    return 4;
  }
}


void receive_task(void *pvParameter){
  while(true){
    if(transmitFlag != true){
        // reset flag
      receivedFlag = false;

      Packet packet;

      // you can also read received data as byte array
      byte byteArr[131];
      int numBytes = radio.getPacketLength();
      Serial.print("Packet length: ");
      Serial.println(numBytes);
      int state = radio.readData(byteArr, numBytes);


      if (state == RADIOLIB_ERR_NONE) {
        // packet was successfully received
        Serial.println(F("[SX1278] Received packet!"));

        contador++;

        memcpy(&packet, byteArr, sizeof(packet));

        //Funcion de leer datos y guardar en buffer
        int flag = leer_datos(sizeof(packet.payload), packet.messageID, packet.senderID, packet.receiverID, packet.payload);

        Serial.print("[SX1278] Message ID: ");
        Serial.println(packet.messageID);
        Serial.print("[SX1278] Sender ID: ");
        Serial.println(packet.senderID);
        Serial.print("[SX1278] Receiver ID: ");
        Serial.println(packet.receiverID);


        //print data of the packet
        Serial.print(F("[SX1278] Payload:\t\t"));
        //print the byte array
        for (int i = 0; i < sizeof(packet.payload); i+=4) {
          float value = *((float*)(packet.payload + i));
          Serial.print(value);
          Serial.print(F(" "));
        }
        Serial.println("");

        //Serial.println(str);

        // print RSSI (Received Signal Strength Indicator)
        // Serial.print(F("[SX1278] RSSI:\t\t"));
        // Serial.print(radio.getRSSI());
        // Serial.println(F(" dBm"));

        // // print SNR (Signal-to-Noise Ratio)
        // Serial.print(F("[SX1278] SNR:\t\t"));
        // Serial.print(radio.getSNR());
        // Serial.println(F(" dB"));

        // // print frequency error
        // Serial.print(F("[SX1278] Frequency error:\t"));
        // Serial.print(radio.getFrequencyError());
        // Serial.println(F(" Hz"));

        //       // print frequency error
        // Serial.print(F("[SX1278] Time on Air:\t"));
        // Serial.print(radio.getTimeOnAir(numBytes));
        // Serial.println(F(" microsecons"));

        //tomadecisiones(flag);

      } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
        // packet was received, but is malformed
        Serial.println(F("[SX1278] CRC error!"));

        //EJECUTAR CASO ESPECIAL DE DATA CORRUPTA PARA AUMENTAR CURRENTPOS Y GUARDAR 0s EN BUFFER DE INTERES
        
        contador_errores++;

      } else {
        // some other error occurred
        Serial.print(F("[SX1278] Failed, code "));
        Serial.println(state);
        contador_errores++;
      }

      Serial.print(F("[SX1278] Contador: "));
      Serial.println(contador);
      Serial.print(F("[SX1278] Contador error: "));
      Serial.println(contador_errores);

      Serial.println("");
    }
    else{
      Serial.println("Se estan enviando datos...");
    }

    vTaskSuspend(NULL);
  }

}


int sendmessage_radiolib(size_t size_data, byte data[]) {

  Serial.print(F("[SX1278] Transmitting packet ... "));

  Packet packet; //Creando packete como estructura Packet

  packet.messageID = count++;
  packet.senderID = 0x1; // Set your sender ID
  packet.receiverID = 0x2; // Set the intended receiver ID
  memcpy(packet.payload, data, size_data); // Copy your byte array into the payload

  // Convert the Packet struct to a byte array
  byte* packetBytes = reinterpret_cast<byte*>(&packet);

  int state = radio.startTransmit(packetBytes, sizeof(packet));

  if (state == RADIOLIB_ERR_NONE) {
    // the packet was successfully transmitted
    Serial.println(F(" success!"));

    //Print the message payload and its length 
    Serial.print(F("[SX1278] Payload:\t\t"));
    Serial.println(packet.payload[0]);

    return 1;

  } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 256 bytes
    Serial.println(F("too long!"));
    return 0;

  } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
    // timeout occurred while transmitting packet
    Serial.println(F("timeout!"));
    return 0;

  } else {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);
    return 0;
  }
}


void send_task(void *pvParameters){
while(1){
  if(transmitFlag == true){
    Serial.print(F("[SX1278] Transmitting packet ... "));

    detachInterrupt(2);

    Packet2 packet;

    byteArr[0] = 0x01;

    Serial.print(F("[SX1278] Payload size:\t\t"));
    Serial.println(sizeof(byteArr));

    if(sendmessage_radiolib(sizeof(byteArr), byteArr)){
      transmitFlag = false;
      Serial.print(F("[SX1278] Starting to listen  for packets again... "));
      attachInterrupt(digitalPinToInterrupt(2), setFlag, RISING);
        int state = radio.startReceive();
        if (state == RADIOLIB_ERR_NONE) {
          Serial.println(F("success!"));
        } else {
          Serial.print(F("failed, code "));
          Serial.println(state);
          //while (true);
        }
    }
    else{
      Serial.print(F("[S1278] Problema al intentar enviar el paquete"));
    }

    packet.messageId = count++;
    packet.senderId = 1; // Set your sender ID
    packet.receiverId = 2; // Set the intended receiver ID
    memcpy(packet.payload, byteArr, sizeof(packet.payload)); // Copy your byte array into the payload

    // Convert the Packet struct to a byte array
    byte* packetBytes = reinterpret_cast<byte*>(&packet);

    int state = radio.startTransmit(packetBytes, sizeof(packet));

      if (state == RADIOLIB_ERR_NONE) {
        // the packet was successfully transmitted
        Serial.println(F(" success!"));

      } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
        // the supplied packet was longer than 256 bytes
        Serial.println(F("muy largo!"));

      } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
        // timeout occurred while transmitting packet
        Serial.println(F("timeout!"));

      } else {
        // some other error occurred
        Serial.print(F("fallo, coidgo "));
        Serial.println(state);
      }

    //Delay para permitir que se termine de enviar el paquete, no poner en modo receptor de inmediato
    delay(500);

    transmitFlag = false;

    Serial.print(F("[SX1278] Comienza a escuchar paquetes otra vez... "));

    int state2 = radio.startReceive();

    attachInterrupt(digitalPinToInterrupt(2), setFlag, RISING); //Reinicia ISR

    if (state2 == RADIOLIB_ERR_NONE) {
      Serial.println(F("Exito!"));
    } else {
      Serial.print(F("fallo, codigo "));
      Serial.println(state2);
      //while (true);
    }
    
    vTaskSuspend(NULL);
  }
}
}

void blink(void *pvParameters){
  while(1){
    //Blink an LED on PIN 12
    digitalWrite(12, HIGH);
    delay(1000);
    digitalWrite(12, LOW);
    delay(1000);
  }
}

void setup_lora(void){
  // initialize SX1278 with default settings
  Serial.print(F("[SX1278] Initializing ... "));
  int state = radio.begin();
 
  radio.setFrequency(433.0);
  radio.setSpreadingFactor(7);
  radio.setBandwidth(250.0);
  // radio.setOutputPower(15);

   //Set pin 2 as interrupt input pin
  pinMode(2, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), setFlag, RISING);

    if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  // start listening for LoRa packets
  Serial.print(F("[SX1278] Starting to listen ... "));
  int state2 = radio.startReceive();
  if (state2 == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state2);
    while (true);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(12, OUTPUT);

  pinMode(32, INPUT);
  attachInterrupt(digitalPinToInterrupt(32), ISR_button, RISING);

  //Pull down for pin 2
  pinMode(2, INPUT_PULLDOWN);

  setup_lora();

  xTaskCreatePinnedToCore(
    send_task, /* Function to implement the task */
    "send_task", /* Name of the task */
    10000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    1,  /* Priority of the task */
    &xHandle_send_task,  /* Task handle. */
    0); /* Core where the task should run */

  vTaskSuspend(xHandle_send_task);

    xTaskCreatePinnedToCore(
    receive_task, /* Function to implement the task */
    "receive_task", /* Name of the task */
    10000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    2,  /* Priority of the task */
    &xHandle_receive_task,  /* Task handle. */
    1); /* Core where the task should run */

    vTaskSuspend(xHandle_receive_task);

    xTaskCreatePinnedToCore(
    blink, /* Function to implement the task */
    "receive_task", /* Name of the task */
    1024,  /* Stack size in words */
    NULL,  /* Task input parameter */
    0,  /* Priority of the task */
    &xHandle_blink,  /* Task handle. */
    1); /* Core where the task should run */
}

void loop() {
  //delay(1000);
}
