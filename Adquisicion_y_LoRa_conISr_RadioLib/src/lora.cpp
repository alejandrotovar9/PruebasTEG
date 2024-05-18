/*
EMISOR DE DATOS LORA
Jose Tovar
06/02/2024
Trabajo Especial de Grado - EIE
*/
#include <lora_header.h>
#include <DAQ.h>

const int csPin = 5;          // LoRa radio chip select
const int resetPin = 14;       // LoRa radio reset
const int irqPin = 2;         // change for your board; must be a hardware interrupt pin

byte msgCount = 0;            // count of outgoing messages
//OJO UNO ES 0xFF y el otro 0xbb, para que tengan diferentes identificadores
byte localAddress = 0xFF;     // address of this device
byte destination = 0xBB;      // destination to send to
long lastSendTime = 0;        // last send time
int interval = 150;          // interval between sends

int expected_length = 1; //Longitud esperada de los datos enviados por el receptor

//Contador de paquetes a enviar en funcion send_packet
int contador_paquetes = 0;

//Variables goblales para excepcion y conteo de errores
int error_count = 0;
int general_count = 0;

//Contador de iteraciones generales, peticiones hasta ahora
int iteraciones_peticiones = 0;

bool datacorrupta = false;

BufferACL trama; //Estructura de datos para recibir la trama de aceleracion

//Objeto ESP32Time
ESP32Time rtc;

//counter to keep track of transmitted packets
byte count_radiolib = 0;

struct Packet {
  byte messageId;
  byte senderId;
  byte receiverId;
  byte payload[128]; // Adjust the size as needed
};

struct Packet2 {
  byte messageID;
  byte senderID;
  byte receiverID;
  byte payload[1]; // Adjust the size as needed
};

//Estructuras para actualizacion de RTC
struct TimePacket{
  byte messageID;
  byte senderID;
  byte receiverID;
  byte payload[24]; // Adjust the size as needed
};

struct StringPacket {
  byte messageID;
  byte senderID;
  byte receiverID; // Message ID
  char payload[19]; // Message (18 characters for "Smart Sensor Ready" + 1 for the null-terminating character)
};

struct THIPacket {
  byte messageID;
  byte senderID;
  byte receiverID;
  time_t timestamp; // Timestamp
  float temperature; // Temperature
  float humidity; // Humidity
  float yaw; // Yaw
  float pitch; // Pitch
  float roll; // Roll
};

struct timestruct{
  int year;
  int month;
  int day;
  int hour;
  int minute;
  int second;
};

union PacketUnion {
  Packet2 packet2;
  TimePacket timePacket;
};

int comando_o_rtc = 0; //Variable para indicar si se recibio un comando o una actualizacion de RTC


// // SX1278 has the following connections:
// // NSS pin:   10
// // DIO0 pin:  2
// // RESET pin: 9
// // DIO1 pin:  3

byte byteArr[128];

SX1278 radio = new Module(5, 2, 14, 3);

//Bandera para indicar paquete recibido
//volatile bool receivedFlag = false;

//Bandera para indicar estado de transmision o recepcion
bool transmitFlag = false;


//USO DE STATIC EVITA EL REBOOT
//Fill this arrays with zeros
static float floatArrayX[(NUM_DATOS)]; //Creacion de float array
static float floatArrayY[(NUM_DATOS)];
static float floatArrayZ[(NUM_DATOS)];

int contador_paquetes_interno = 0;
float* selectedFloatArray;

//Bandera para indicar paquete recibido
volatile bool receivedFlag = false;

//Interrupt Service Routine
void ICACHE_RAM_ATTR setFlag(void){
  //Activo tarea de recepcion de datos
  vTaskResume(xHandle_receive_task);
  //transmitFlag = true;
  receivedFlag = true;
}

/*Funcion para generar el arreglo de chunks que contienen 128 bytes cada uno para su posterior envio, mediante una cola, a la tarea de envio de datos LoRa en forma de byte array.

Output -> int (1 o 0 dependiendo de si se envio exitosamente la cola)
Input -> int contador_paquetes (para decidir cual eje enviar dependiendo de si ya se enviaron todos los paquetes del eje anterior)
*/
int generararray(int contador_paquetes)
{

    //Recibiendo cola con datos de aceleracion en estructura de datos 
    //Los datos son un float array dentro del buffer
    if(contador_paquetes == 0){
          const int chunkSize = 64;
          contador_paquetes_interno = 0; //REINICIO CONTADOR INTERNO INDEPENDIENTE DE SI ENTRO POR PRIMERA VEZ

          if(xQueueReceive(tramaLoRaQueue, &trama, portMAX_DELAY))
          {
            //Evita el error por MeditationGuru en Core0
            //Copia los datos por partes en vez de completos, evita problemas de reboot
            for (int i = 0; i < NUM_DATOS; i += chunkSize)
            {
                // Calculate the size of the current chunk
                int currentChunkSize = min(chunkSize, NUM_DATOS - i);

                // Copy and process the chunk
                memcpy(floatArrayX + i, trama.bufferX + i, currentChunkSize * sizeof(float));
                memcpy(floatArrayY + i, trama.bufferY + i, currentChunkSize * sizeof(float));
                memcpy(floatArrayZ + i, trama.bufferZ + i, currentChunkSize * sizeof(float));

                // Process the data in floatArrayX, floatArrayY, and floatArrayZ here
            }
            /*The memcpy function takes three arguments: the destination pointer, the source pointer, and the number of bytes to copy.*/
          }
          selectedFloatArray = floatArrayX;
    }
    else if(contador_paquetes == NUM_DATOS/CHUNK_SIZE) //32/64/128...
    {
          contador_paquetes_interno = 0;
          selectedFloatArray = floatArrayY;
    }
    else if(contador_paquetes == NUM_DATOS/CHUNK_SIZE*2) // (NUM_DATOS/CHUNKSIZE - 1)*2
    {
          contador_paquetes_interno = 0;
          selectedFloatArray = floatArrayZ;
    }

    // Calculate the number of chunks
    int numChunks = sizeof(floatArrayX) / sizeof(float) / CHUNK_SIZE; //El numero de chunks indica el numero de bytes final
    //Print the amount of chunks
    Serial.print("Amount of chunks calculated: ");
    Serial.println(numChunks);
    

    // Check if the size of floatArray is divisible by CHUNK_SIZE
    if (NUM_DATOS / sizeof(float) % CHUNK_SIZE != 0) {
        Serial.println("Error: Size of floatArray is not divisible by CHUNK_SIZE");
    }
    // Create an array to hold the chunks
    float chunks[numChunks][CHUNK_SIZE];

    // Split the array into chunks
    for (int i = 0; i < numChunks; i++) {
      memcpy(chunks[i], &selectedFloatArray[i * CHUNK_SIZE], CHUNK_SIZE * sizeof(float)); //Copiar 128bytes de Float Array (a partir de la posicion especificada por i) en chunks
    }

    if(xQueueSend(arrayQueue, &chunks[contador_paquetes_interno], portMAX_DELAY) == pdTRUE){
      Serial.println("Se envio la cola a la tarea de envio LoRa.");
    }
    else{
      Serial.println("Problema al enviar cola...");
      return 0;
    }

    contador_paquetes_interno++;

    return 1;
}

//Funcion para enviar temperatura humedad e inclinacion
int send_thi_radiolib(time_t tstamp, float temp, float hum, float yaw, float pitch, float roll){
  if(transmitFlag){
    Serial.print(F("[SX1278] Transmitting T.H.I ... "));

    THIPacket packet; //Creando paquete como estructura Packet

    packet.messageID = 200;
    packet.senderID = 0x2; // Set your sender ID
    packet.receiverID = 0x1; // Set the intended receiver ID
    packet.timestamp = tstamp;
    packet.temperature = temp;
    packet.humidity = hum;
    packet.yaw = yaw;
    packet.pitch = pitch;
    packet.roll = roll;
    //memcpy(packet.payload, data, size_data); // Copy your byte array into the payload

    // Convert the Packet struct to a byte array
    byte* packetBytes = reinterpret_cast<byte*>(&packet);

    int state = radio.startTransmit(packetBytes, sizeof(packet));

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
    return 1;
  }
  else{
      Serial.println("Data is being received");
      return 0;
  }  
}

//Funcion para enviar mensaje de i'm alive
int send_imalive_radiolib(size_t size_data, byte data[]) {
  if(transmitFlag){
    Serial.print(F("[SX1278] Transmitting i'm alive ... "));

    StringPacket packet; //Creando paquete como estructura Packet

    packet.messageID = 0xFF;
    packet.senderID = 0x2; // Set your sender ID
    packet.receiverID = 0x1; // Set the intended receiver ID
    memcpy(packet.payload, data, size_data); // Copy your byte array into the payload

    // Convert the Packet struct to a byte array
    byte* packetBytes = reinterpret_cast<byte*>(&packet);

    int state = radio.startTransmit(packetBytes, sizeof(packet));

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
    return 1;
  }
  else{
      Serial.println("Data is being received");
      return 0;
  }  
}

int sendmessage_radiolib(size_t size_data, byte data[]) {
  if(transmitFlag){
    Serial.print(F("[SX1278] Transmitting packet ... "));

    Packet packet; //Creando paquete como estructura Packet

    packet.messageId = count_radiolib;

    if(count_radiolib < (((NUM_DATOS * 4))*3 / 128) - 1){
      count_radiolib++;
    }
    else{
      count_radiolib = 0;
    }

    packet.senderId = 0xFF; // Set your sender ID
    packet.receiverId = 0xBB; // Set the intended receiver ID
    memcpy(packet.payload, data, size_data); // Copy your byte array into the payload

    // Convert the Packet struct to a byte array
    byte* packetBytes = reinterpret_cast<byte*>(&packet);

    int state = radio.startTransmit(packetBytes, sizeof(packet));

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
    return 1;
  }
  else{
      Serial.println("Data is being received");
      return 0;
  }
}

/*Toma como entrada el byte array que contiene la informacion del RTC*/
void updateRTC(byte* packetBytes, size_t length) {

   // Save the old timezone
    char *oldTZ = getenv("TZ");

    // Set the timezone to UTC
    setenv("TZ", "UTC", 1);
    tzset();

  // Convert the byte array back to a TimePacket
  timestruct* packet = reinterpret_cast<timestruct*>(packetBytes);

  // Extract the date and time from the packet
  int year = packet->year;
  int month = packet->month;
  int day = packet->day;
  int hour = packet->hour;
  int minute = packet->minute;
  int second = packet->second;

  // Set the time
  struct tm timeinfo;
  timeinfo.tm_year = year - 1900;
  timeinfo.tm_mon = month - 1;
  timeinfo.tm_mday = day;
  timeinfo.tm_hour = hour;
  timeinfo.tm_min = minute;
  timeinfo.tm_sec = second;

  // Convert the tm structure to time_t
  time_t t = mktime(&timeinfo);
  Serial.println(t);  // Print the result from mktime

  // Create a timeval structure
  struct timeval now = { .tv_sec = t };

  // Update the RTC
  settimeofday(&now, NULL);

      // Restore the old timezone
    if (oldTZ) {
        setenv("TZ", oldTZ, 1);
    } else {
        unsetenv("TZ");
    }
    tzset();

  Serial.println("Time and date updated on Smart Sensor!");

  // Print the current time from the time_packet structure
    Serial.print("Year: ");
    Serial.println(timeinfo.tm_year + 1900);
    Serial.print("Month: ");
    Serial.println(timeinfo.tm_mon + 1);
    Serial.print("Day: ");
    Serial.println(timeinfo.tm_mday);
    Serial.print("Hour: ");
    Serial.println(timeinfo.tm_hour);
    Serial.print("Minute: ");
    Serial.println(timeinfo.tm_min);
    Serial.print("Second: ");
    Serial.println(timeinfo.tm_sec);
}

void receive_task(void *pvParameter){
  while(true){
      //receivedFlag = false;

      PacketUnion packetUnion;
      byte byteArr[sizeof(TimePacket)]; // Make sure the byte array is large enough for the largest packet

      int numBytes = radio.getPacketLength(); //Tamaño del paquete recibido

      if(numBytes == sizeof(TimePacket)){
        Serial.println("Se recibio una actualizacion de RTC.");
        comando_o_rtc = 1;
      }
      else if(numBytes <= sizeof(Packet2)){
        Serial.println("Se recibio un comando.");
        comando_o_rtc = 0;
      }
      else{
        Serial.println("Se recibio algo que no es un comando ni una actualizacion de RTC.");
        datacorrupta = true;
        //vTaskSuspend(NULL); // Ignore if it's larger than TimePacket
      }
      
      Serial.print("Packet length: ");
      Serial.println(numBytes);
      int state = radio.readData(byteArr, numBytes);

      if(state == RADIOLIB_ERR_NONE && datacorrupta == false){
        Serial.println(F("[SX1278] Paquete recibido desde estacion base!"));

        //Pointer para apuntar a la estructura escogida
        void* paquete_received;

        if(comando_o_rtc == 1){
          //Es una actualizacion RTC
           memcpy(&packetUnion.timePacket, byteArr, sizeof(packetUnion.timePacket));

           paquete_received = &packetUnion.timePacket;

            //Print message ID, sender ID, receiver ID and payload
            Serial.print("[SX1278] Message ID: "); 
            Serial.println(packetUnion.timePacket.messageID);
            Serial.print("[SX1278] Sender ID: ");
            Serial.println(packetUnion.timePacket.senderID);
            Serial.print("[SX1278] Receiver ID: ");
            Serial.println(packetUnion.timePacket.receiverID);

            if(packetUnion.timePacket.messageID == 255){
              //Es actualizacion de RTC, actualizo en del ESP32 con la info del payload
              updateRTC(packetUnion.timePacket.payload, sizeof(packetUnion.timePacket.payload));
            }
        }
        else if(comando_o_rtc == 2){
          continue; //CASO DE PAQUETE NO ESPERADO
        }
        else{
          //SE RECIBIO UN COMANDO
            memcpy(&packetUnion.packet2, byteArr, sizeof(packetUnion.packet2));
            paquete_received = &packetUnion.packet2;

            Serial.print("[SX1278] Message ID: "); 
            Serial.println(packetUnion.packet2.messageID);
            Serial.print("[SX1278] Sender ID: ");
            Serial.println(packetUnion.packet2.senderID);
            Serial.print("[SX1278] Receiver ID: ");
            Serial.println(packetUnion.packet2.receiverID);

            if(packetUnion.packet2.payload[0] == 0x01){
              flag_acl = true; //Se requiere paquete de forma inmediata
              globalTimestamp = getEpochTime();
            }
          //Es un comando
        }
      }
      else if(state == RADIOLIB_ERR_CRC_MISMATCH){
        Serial.println("[SX1278] CRC Error!");
      }
      else if (datacorrupta)
      {
        // some other error occurred
        Serial.print(F("[SX1278] Data corrupta..."));
        datacorrupta = false; ///Reinicio bandera
      }
      else{
        Serial.print("[SX1278] Fallo. Code: ");
        Serial.println(state);
      }

      vTaskSuspend(NULL); //Se suspende a si misma hasta el proximo mensaje
    }
}

//Funcion para enviar confirmacion de setup listo y comenzar sincronizacion de RTC
void send_imalive(void){
    transmitFlag = true;

    String str = "Smart Sensor Ready"; // Your string here
    size_t size_data = str.length() + 1; // +1 for the null-terminating character

    // Convert the string to a byte array
    byte data[size_data];
    str.getBytes(data, size_data);

    // Send the byte array
    send_imalive_radiolib(size_data, data);
    Serial.println("Sending string!");
    Serial.println();

    delay(500);

    //Start listening to packets again

    transmitFlag = false;
}

//Funcion para enviar THI
void send_thi(void){
    transmitFlag = true;

    BMEData temp_hum;
    IncData inc;

    if(xQueueReceive(temphumarrayQueue, &temp_hum, portMAX_DELAY)){
      Serial.println("Se recibio la cola con los datos de temperatura y humedad final");
    }

    if(xQueueReceive(incarrayQueue, &inc, portMAX_DELAY)){
      Serial.println("Se recibio la cola con los datos de inclinacion");
    }
  
    //Print all the inpust to the send_thi_radiolib
    // Serial.println(globalTimestamp);
    // Serial.println(temp_hum.temperature);
    // Serial.println(temp_hum.humidity);
    // Serial.println(inc.IncYaw);
    // Serial.println(inc.IncPitch);
    // Serial.println(inc.IncRoll);

    // Send the byte array
    send_thi_radiolib(globalTimestamp, temp_hum.temperature, temp_hum.humidity, inc.IncYaw, inc.IncPitch, inc.IncRoll);
    Serial.println("Sending THI package!");
    Serial.println();

    delay(500);

    //Start listening to packets again

    transmitFlag = false;
}

//PRUEBA
void send_packet(void *pvParameters){
  while(1){
    transmitFlag = true;

    //detach interrupt from pin 2
    detachInterrupt(2);

    Serial.print("Contador de paquetes actual antes de entrar en generararray: ");
    Serial.println(contador_paquetes);

    // if(contador_paquetes >= ((NUM_DATOS * 4))*3 / 128){
    //   contador_paquetes = 0;
    // }

    generararray(contador_paquetes); //Genero el array y mando un chunk, dependiendo del contador

    
    if(contador_paquetes < (((NUM_DATOS * 4))*3 / 128)){
         contador_paquetes++; //Aumento el contador para enviar el siguiente paquete en el proximo envio
    }

     float floatarray[CHUNK_SIZE];
     byte data[CHUNK_SIZE * sizeof(float)]; //Inicializacion de byte array

     if(xQueueReceive(arrayQueue, &floatarray, portMAX_DELAY)){
        Serial.println("Se recibio la cola con los datos");
     }

    // //Debo recibir los datos a enviar en una cola y formatearlos de ser necesario
    
    //Convert the float array to a byte array
     memcpy(data, floatarray, sizeof(floatarray)); //CHUNK_SIZE * sizeof(float)
    
    size_t size_data;

    size_data = sizeof(data);

    //Llamo a la funcion que crea la trama de datos (payload)
    sendmessage_radiolib(size_data, data);
    Serial.println("Sending byte array con datos!");
    Serial.println();

    //Espero X segundos luego de enviar mensaje
    vTaskDelay(interval/portTICK_PERIOD_MS);

    //Suspendo esta tarea hasta que se reciba otro mensaje
    if(contador_paquetes >= (((NUM_DATOS * 4))*3 / 128)){
      
      promediofinal_inc();
      promediofinal_temphum();
      send_thi();

      iteraciones_peticiones++;
      Serial.print("Iteraciones: ");
      Serial.println(iteraciones_peticiones);

      if(iteraciones_peticiones == 2){
        Serial.println("Reiniciando micro...");
        esp_restart();
      }

      //AQUI SE PUEDE ENVIAR TEMPERATURA, HUMEDAD Y TIMESTAMP INICIAL EN OTRO PAQUETE


      Serial.println("Reactivando tareas de adquisicion de datos!");
      contador_paquetes = 0;
      vTaskResume(xHandle_leerDatosACL);
      vTaskResume(xHandle_readBMETask);
      vTaskResume(xHandle_readMPU9250);

        Serial.print("Memoria disponible en send task: ");
        Serial.println(ESP.getFreeHeap());
        Serial.println(ESP.getFreePsram());
        Serial.println(uxTaskGetStackHighWaterMark(NULL));

      transmitFlag = false; //Se desactiva el modo envio y se comienza a escuchar otra vez
        // start listening for LoRa packets
      Serial.print(F("[SX1278] Starting to listen for commands again... "));
      attachInterrupt(digitalPinToInterrupt(2), setFlag, RISING);
      int state = radio.startReceive();
      if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("success!"));
      } 
      else{
        Serial.print(F("failed, code "));
        Serial.println(state);
        //while (true);
      }

      vTaskSuspend(NULL);
    }
  }  
}

void setup_lora_radiolib() {

  // initialize SX1278 with default settings
  Serial.print(F("[SX1278] Initializing ... "));
  int state = radio.begin();
  radio.setSpreadingFactor(7);
  radio.setBandwidth(250.0);
  // radio.setOutputPower(15);

  //radio.setPacketReceivedAction(setFlag);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("LoRa RadioLib init success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  //ENVIO MENSAJE DE INICIALIZACION CORRECTA A ESTACION BASE
  send_imalive();

  //Interrupciones
  //Se configura pin 2 para manejar interrupcion del SX1278
    pinMode(2, INPUT);
    attachInterrupt(digitalPinToInterrupt(2), setFlag, RISING);

  // start listening for LoRa packets
  Serial.print(F("[SX1278] Starting to listen for commands... "));
  int state2 = radio.startReceive();
  if (state2 == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state2);
    while (true);
  }
}
