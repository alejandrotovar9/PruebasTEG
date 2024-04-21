#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <RadioLib.h>
#include <WiFi.h>
#include <wifi_credentials.h>
#include <time.h>
#include <ESP32Time.h>


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
  byte messageID;
  byte senderID;
  byte receiverID;
  byte payload[1]; // Adjust the size as needed
};

struct TimePacket{
  byte messageID;
  byte senderID;
  byte receiverID;
  byte payload[30]; // Adjust the size as needed
};

struct timestruct{
  int year;
  int month;
  int day;
  int hour;
  int minute;
  int second;
};

struct StringPacket {
  byte receiverID;
  byte senderID;
  byte messageID; // Message ID
  char payload[19]; // Message (18 characters for "Smart Sensor Ready" + 1 for the null-terminating character)
};

union PacketUnion {
  Packet packet1;
  Packet2 packet2;
  TimePacket timePacket;
  StringPacket stringPacket;
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

// Para guardar el buffer actual en donde se esta guardando
float *bufferactual;

//Flag para controlar que tipo de comando se esta enviando 

/* 
1- Envio de datos de forma inmediata
2 - Actualizacion de RTC tras inicializacion
*/
volatile int comando = 0;

byte byteArr[1];

//RTC
ESP32Time rtc;
const char* ntpServer = "0.north-america.pool.ntp.org"; //0.north-america.pool.ntp.org FUNCIONA CON RED RAPIDA

/*
0.north-america.pool.ntp.org
time.google.com - Google's NTP server.
time.windows.com - Microsoft's NTP server.
time.nist.gov - NIST's NTP server.
time.apple.com - Apple's NTP server.
time.cloudflare.com - Cloudflare's NTP server
*/
//CON REDES LENTAS NO FUNCIONA BIEN

/*Caracas, Venezuela está en la zona horaria GMT-4 */
const long  gmtOffset_sec = -14400; //-4*60*60
const int   daylightOffset_sec = 0;


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

// volatile unsigned long last_interrupt_time = 0;

// //ISR con antirebote por Software
// void ISR_button() {
//   unsigned long interrupt_time = millis();
//   // If interrupts come faster than 200ms, assume it's a bounce and ignore
//   if (interrupt_time - last_interrupt_time > 200) {
//     transmitFlag = true;
//     vTaskResume(xHandle_send_task);
//   }
//   last_interrupt_time = interrupt_time;
// }


#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void ICACHE_RAM_ATTR setFlag(void) {
  // we got a packet, set the flag
  comando = 1;
  vTaskResume(xHandle_receive_task);
  receivedFlag = true;
}


void fillBuffer(float* buffer, unsigned char* byteArray, int size, bool corrupted_data_flag) {
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

        if(corrupted_data_flag){
            //En caso de recibir un paquete corrupto, llenar ese espacio en buffer con 0s y continuar en siguiente posicion (32 flotantes despues) para el siguiente paquete
            buffer[currentPos] = 0.0f;
        }
        else{
          //Copiando 4 bytes del bytearray para convertir a flotante
          for (int j = 0; j < 4; j++) {
              u.b[j] = byteArray[i + j];
          }
        
          // append float to buffer and increase position
          buffer[currentPos] = u.f;
        }
        
        //aumentar la posicion siempre
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

    //Guardar incoming en buffer
  if((int)incomingMsgId <= 31){
    Serial.println("Guardando en bufferX!!!");
    //currentPos = 0;
    bufferactual = buffer_prueba.bufferX;
    fillBuffer(buffer_prueba.bufferX, incoming, packetSize, false);  
  }
  else if((int)incomingMsgId > 31  && (int)incomingMsgId <= 63){
    Serial.println("Guardando en bufferY!!!");
    //currentPos = 0;
    bufferactual = buffer_prueba.bufferY;
    fillBuffer(buffer_prueba.bufferY, incoming, packetSize, false);  
  }
  else if((int)incomingMsgId > 63 && (int)incomingMsgId <= 95){
    Serial.println("Guardando en bufferZ!!!");
    //currentPos = 0;
    bufferactual = buffer_prueba.bufferZ;
    fillBuffer(buffer_prueba.bufferZ, incoming, packetSize, false);  
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

int data_o_comando = 0;

void receive_task(void *pvParameter){
  while(true){
    if(transmitFlag != true){
        // reset flag
      receivedFlag = false;

      PacketUnion packetUnion;

      // Packet packet;

      // you can also read received data as byte array
      byte byteArr[131]; //El de mayor tamaño
      int numBytes = radio.getPacketLength();

      if(numBytes == 22){
        Serial.println("Se recibio una actualizacion de SmartSensor.");
        data_o_comando = 1;
      }
      else if(numBytes > 22){
        Serial.println("Se recibieron datos del SS.");
        data_o_comando = 0;
      }
      else{
        Serial.println("Se recibio algo que no es un comando ni una actualizacion de RTC.");
        vTaskSuspend(NULL); // Ignore if it's larger than TimePacket
      }

      Serial.print("Packet length: ");
      Serial.println(numBytes);
      int state = radio.readData(byteArr, numBytes);

      if (state == RADIOLIB_ERR_NONE) {
        // packet was successfully received
        Serial.println(F("[SX1278] Paquete recibido!"));

        contador++;

        if(data_o_comando == 0){
          //Es actualizacion
          memcpy(&packetUnion.stringPacket, byteArr, sizeof(packetUnion.timePacket));
          if(packetUnion.stringPacket.messageID == 255){
            comando = 2;
            //vTaskResume(xHandle_send_task);
              //Es actualizacion de RTC
        }
        else{
          //Son datos
          memcpy(&packetUnion.packet1, byteArr, sizeof(packetUnion.packet1));

          Serial.print("[SX1278] Message ID: ");
          Serial.println(packetUnion.packet1.messageID);
          Serial.print("[SX1278] Sender ID: ");
          Serial.println(packetUnion.packet1.senderID);
          Serial.print("[SX1278] Receiver ID: ");
          Serial.println(packetUnion.packet1.receiverID);

          //print data of the packet
          Serial.print(F("[SX1278] Payload:\t\t"));
          //print the byte array
          for (int i = 0; i < sizeof(packetUnion.packet1.payload); i+=4) {
            float value = *((float*)(packetUnion.packet1.payload + i));
            Serial.print(value);
            Serial.print(F(" "));
          }
          Serial.println("");

          int flag = leer_datos(sizeof(packetUnion.packet1.payload), packetUnion.packet1.messageID, packetUnion.packet1.senderID, packetUnion.packet1.receiverID, packetUnion.packet1.payload);
        }
      } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
        // packet was received, but is malformed
        Serial.println(F("[SX1278] CRC error!"));

        //EJECUTAR CASO ESPECIAL DE DATA CORRUPTA PARA AUMENTAR CURRENTPOS Y GUARDAR 0s EN BUFFER DE INTERES
        if(numBytes == 131){
          fillBuffer(bufferactual, NULL, sizeof(packetUnion.packet1.payload), true);
        }
        else{
          Serial.println("[SX1278] La data recibida esta corrupta y no es para este receptor");
        }
        
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
}


int sendmessage_radiolib(size_t size_data, byte data[]) {

  Serial.print(F("[SX1278] Transmitting packet ... "));

  //Crea una union de tipo packetunion para escoger el tipo de estructura de datos a utilizarse dependiendo del payload
  PacketUnion packetUnion;

  //Declaradas aqui para luego asignarles un valor en el if-else
  byte* packetBytes;
  size_t packetSize;

  if(size_data == 1){ //Es un comando
    packetUnion.packet2.messageID = count++;
    packetUnion.packet2.senderID = 0x1; // Set your sender ID
    packetUnion.packet2.receiverID = 0x2; // Set the intended receiver ID
    memcpy(packetUnion.packet2.payload, data, size_data); // Copy your byte array into the payload

    //Print the message payload and its length 
    Serial.print(F("[SX1278] Payload:\t\t"));
    Serial.println(packetUnion.packet2.payload[0]);

    // Convert the Packet2 struct to a byte array
    byte* packetBytes = reinterpret_cast<byte*>(&packetUnion.packet2);
    packetSize = sizeof(packetUnion.packet2);
    // Send packetBytes using RadioLib
  }
  else{
    //Es actualizacion de RTC
    packetUnion.timePacket.messageID = 0xFF; //messageID designado para comandos
    packetUnion.timePacket.senderID = 0x1; // Set your sender ID
    packetUnion.timePacket.receiverID = 0x2; // Set the intended receiver ID
    memcpy(packetUnion.timePacket.payload, data, size_data); // Copy your byte array into the payload

    //Print the message payload and its length 
    Serial.print(F("[SX1278] Payload:\t\t"));
    Serial.println(packetUnion.timePacket.payload[0]);

    // Convert the TimePacket struct to a byte array
    byte* packetBytes = reinterpret_cast<byte*>(&packetUnion.timePacket);
    packetSize = sizeof(packetUnion.timePacket);
    // Send packetBytes using RadioLib
  }

  int state = radio.startTransmit(packetBytes, packetSize);

  if (state == RADIOLIB_ERR_NONE) {
    // the packet was successfully transmitted
    Serial.println(F(" success!"));
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

void update_timepacket(void){
    struct tm timeinfo = rtc.getTimeStruct();
    timestruct time_packet;
    time_packet.year = timeinfo.tm_year + 1900;
    time_packet.month = timeinfo.tm_mon + 1;
    time_packet.day = timeinfo.tm_mday;
    time_packet.hour = timeinfo.tm_hour;
    time_packet.minute = timeinfo.tm_min;
    time_packet.second = timeinfo.tm_sec;

    //Print the current time from the time_packet structure
    // Print the current time from the time_packet structure
    Serial.print("Year: ");
    Serial.println(time_packet.year);
    Serial.print("Month: ");
    Serial.println(time_packet.month);
    Serial.print("Day: ");
    Serial.println(time_packet.day);
    Serial.print("Hour: ");
    Serial.println(time_packet.hour);
    Serial.print("Minute: ");
    Serial.println(time_packet.minute);
    Serial.print("Second: ");
    Serial.println(time_packet.second);

}


void printLocalTime()
{
  Serial.println(rtc.getTime("%A, %B %d %Y %H:%M:%S"));
  struct tm timeinfo = rtc.getTimeStruct();
}

void send_task(void *pvParameters){
while(1){
  if(transmitFlag == true){
    Serial.print(F("[SX1278] Transmitting packet ... "));

    detachInterrupt(2);

    //Packet2 packet;

    switch(comando){
      case 1: 
        Packet2 packet;
        byteArr[0] = 0x01; //Enviame datos desde estacion base de forma inmediata

        if(sendmessage_radiolib(sizeof(byteArr), byteArr)){
        Serial.println(F("[SX1278] Packet sent!..."));
        }
        else{
          Serial.print(F("[S1278] Problema al intentar enviar el paquete"));
        }
        break;

      case 2: 
        timestruct time_packet;
        // Fill the time_packet structure with current time
        //update_timepacket();
        struct tm timeinfo = rtc.getTimeStruct();
        time_packet.year = timeinfo.tm_year + 1900;
        time_packet.month = timeinfo.tm_mon + 1;
        time_packet.day = timeinfo.tm_mday;
        time_packet.hour = timeinfo.tm_hour;
        time_packet.minute = timeinfo.tm_min;
        time_packet.second = timeinfo.tm_sec;

        // Convert the time_packet structure to a byte array
        byte *byteArr = (byte *)&time_packet;

        Serial.print(F("El tamaño del byte array de tiempo es: "));
        Serial.println(sizeof(byteArr));

        //Envio el bytearray con el tiempo y la hora en forma de la estructura TimePacket
        if(sendmessage_radiolib(sizeof(byteArr), byteArr)){
        Serial.println(F("[SX1278] Packet sent!..."));
        }
        else{
          Serial.print(F("[S1278] Problema al intentar enviar el paquete"));
        }

        comando=1; //Reinicias comando

        break;
    }
    
    // Serial.print(F("[SX1278] Payload size:\t\t"));
    // Serial.println(sizeof(byteArr));

    // packet.messageId = count++;
    // packet.senderId = 1; // Set your sender ID
    // packet.receiverId = 2; // Set the intended receiver ID
    // memcpy(packet.payload, byteArr, sizeof(packet.payload)); // Copy your byte array into the payload

    // // Convert the Packet struct to a byte array
    // byte* packetBytes = reinterpret_cast<byte*>(&packet);

    // int state = radio.startTransmit(packetBytes, sizeof(packet));

    //   if (state == RADIOLIB_ERR_NONE) {
    //     // the packet was successfully transmitted
    //     Serial.println(F(" success!"));

    //   } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    //     // the supplied packet was longer than 256 bytes
    //     Serial.println(F("muy largo!"));

    //   } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
    //     // timeout occurred while transmitting packet
    //     Serial.println(F("timeout!"));

    //   } else {
    //     // some other error occurred
    //     Serial.print(F("fallo, coidgo "));
    //     Serial.println(state);
    //   }

    //Delay para permitir que se termine de enviar el paquete, no poner en modo receptor de inmediato
    delay(500);

    transmitFlag = false;

    Serial.print(F("[SX1278] Comienza a escuchar paquetes otra vez... \n"));

    update_timepacket();

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

// void printLocalTime()
// {
//   struct tm timeinfo;
//   // Wait until time has been set
//   int retry = 0;
//   while (getLocalTime(&timeinfo,3000) == 0 && retry < 10) {
//       Serial.println("Waiting for time to be set...");
//       ++retry;
//       delay(1000);
//   }

//   if (retry < 10) {
//       printLocalTime();
//   } else {
//       Serial.println("Failed to set time");
//   }
// }

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

void setup_wifi(void){

  // We start by connecting to a WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

}

void setup() {
  Serial.begin(115200);

  pinMode(12, OUTPUT);

  pinMode(32, INPUT);
  attachInterrupt(digitalPinToInterrupt(32), ISR_button, RISING);

  //Pull down for pin 2
  pinMode(2, INPUT_PULLDOWN);

  setup_wifi();

    // Init and get the time
    /*---------set with NTP---------------*/
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    struct tm timeinfo;

    if (getLocalTime(&timeinfo)){
      rtc.setTimeStruct(timeinfo); 
    }

    update_timepacket();
    //printLocalTime(); //Imprime hora actual tras actualizacion por NTP

  // configTime(gmtOffset_sec, daylightOffset_sec, ntpServer, "time.nist.gov", "time.google.com");
  // printLocalTime();

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
