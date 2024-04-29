#include <lora_header.h>

static BufferACL buffer_prueba;

// OJO UNO ES 0xFF y el otro 0xbb, para que tengan diferentes identificadores
byte localAddress = 0xBB; // address of this device
byte destination = 0xFF;  // destination to send to

int NUM_PAQUETES_ESPERADOS = 96;
int expected_length = 128;

// Variables goblales para excepcion y conteo de errores
int error_count = 0;
int general_count = 0;

// Flag para controlar envio de datos
int flag_envio = 1;

// Prueba modo de op
int modo_de_operacion = 1;

// Contadores para funcion de recepcion de datos
int contador_errores = 0;
long int contador = 0;

// counter to keep track of transmitted packets
int count = 0;

// Para guardar el buffer actual en donde se esta guardando
float *bufferactual;

// Flag para controlar que tipo de comando se esta enviando

/*
0- comando
1 - datos
*/
int data_o_comando = 0;
bool datacorrupta = false;

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

#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void ICACHE_RAM_ATTR setFlag(void)
{
  // we got a packet, set the flag
  vTaskResume(xHandle_receive_task);
  receivedFlag = true;
}


void fillBuffer(float *buffer, unsigned char *byteArray, int size, bool corrupted_data_flag)
{
  static int currentPos = 0; // keep track of current position in bufferX as static to prevent changes from function calls

  Serial.print("El valor actual de currentPos en la funcion fillBuffer> ");
  Serial.println(currentPos);

  for (int i = 0; i < size; i += 4)
  {
    // convert 4 bytes to float

    // creando la union
    union
    {
      float f;
      unsigned char b[4];
    } u;

    if (corrupted_data_flag)
    {
      // En caso de recibir un paquete corrupto, llenar ese espacio en buffer con 0s y continuar en siguiente posicion (32 flotantes despues) para el siguiente paquete
      buffer[currentPos] = 0.0f;
    }
    else
    {
      // Copiando 4 bytes del bytearray para convertir a flotante
      for (int j = 0; j < 4; j++)
      {
        u.b[j] = byteArray[i + j];
      }

      // append float to buffer and increase position
      buffer[currentPos] = u.f;
    }

    // aumentar la posicion siempre
    //  if bufferX is full, reset currentPos to 0
    if (currentPos == NUM_DATOS - 1)
    {
      Serial.println("Reinicie currentPos");
      currentPos = 0;
    }
    else
    {
      currentPos++;
    }
  }
}

// Funcion para leer y verificar datos recibidos luego del polling
int leer_datos(size_t packetSize, byte incomingMsgId, byte sender, byte recipient, byte *incoming)
{

  if (packetSize == 0)
    return 0; // if there's no packet, return 0

  if (packetSize != expected_length)
  { // check length for error
    Serial.println("error: message length does not match expected length");
    return 1; // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xBB)
  {
    Serial.println("Este mensaje no es para mi.");
    return 2; // skip rest of function
  }

  // Guardar incoming en buffer
  if ((int)incomingMsgId <= 31)
  {
    Serial.println("Guardando en bufferX!!!");
    // currentPos = 0;
    bufferactual = buffer_prueba.bufferX;
    fillBuffer(buffer_prueba.bufferX, incoming, packetSize, false);
  }
  else if ((int)incomingMsgId > 31 && (int)incomingMsgId <= 63)
  {
    Serial.println("Guardando en bufferY!!!");
    // currentPos = 0;
    bufferactual = buffer_prueba.bufferY;
    fillBuffer(buffer_prueba.bufferY, incoming, packetSize, false);
  }
  else if ((int)incomingMsgId > 63 && (int)incomingMsgId <= 95)
  {
    Serial.println("Guardando en bufferZ!!!");
    // currentPos = 0;
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

  // printf("El valor actual de msgID en int es: %d \n", int(incomingMsgId));

  if (int(incomingMsgId) < NUM_PAQUETES_ESPERADOS - 1) // Va del 0 al 31
  {
    return 3;
  }
  else
  {
    // Se recibieron todos los paquetes
    return 4;
  }
}

void receive_task(void *pvParameter)
{
  while (true)
  {
    if (transmitFlag != true)
    {
      // reset flag
      receivedFlag = false;

      PacketUnion packetUnion;

      // Packet packet;

      // you can also read received data as byte array
      int numBytes = radio.getPacketLength();
      byte byteArr[numBytes]; // El de mayor tamaño

      // Serial.print("Valor actual de datacomando> ");
      // Serial.println(data_o_comando);

      if (numBytes == 22)
      {
        Serial.println("Se recibio una actualizacion de SmartSensor.");
        data_o_comando = 1;
      }
      else if (numBytes == 131)
      {
        Serial.println("Se recibieron datos del SS.");
        data_o_comando = 0;
      }
      else if (numBytes == 28)
      {
        Serial.println("Se recibieron datos de temp y humedad.");
        data_o_comando = 2;
      }
      else
      {
        Serial.println("Se recibio algo que no es un comando ni una actualizacion de RTC.");
        datacorrupta = true;
        //vTaskSuspend(NULL); // Ignore if it's larger than TimePacket
      }

      Serial.print("Packet length: ");
      Serial.println(numBytes);
      int state = radio.readData(byteArr, numBytes);

      if (state == RADIOLIB_ERR_NONE || state == 0 && datacorrupta == false)
      {
        // packet was successfully received
        Serial.println(F("[SX1278] Paquete recibido!"));

        // Serial.print("Valor actual de datacomando> ");
        // Serial.println(data_o_comando);

        contador++;

        switch(data_o_comando)
        {
          case 0:
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

              leer_datos(sizeof(packetUnion.packet1.payload), packetUnion.packet1.messageID, packetUnion.packet1.senderID, packetUnion.packet1.receiverID, packetUnion.packet1.payload);

              //CAMBIAR 95 A NUMDATOS OJOOOOOOOOOOOOOO
              if(packetUnion.packet1.messageID == 95){
                if(xQueueSend(xQueueBufferACL, &buffer_prueba, portMAX_DELAY)){
                  Serial.println("Se envio la estructura bufferprueba a la cola xQueueBufferACL");
                  //vTaskResume(xHandle_send_mqtt);
                }
                else{
                  Serial.println("No se pudo enviar la estructura bufferprueba a la cola xQueueBufferACL");
                }
              }

              break;

          case 1:
              memcpy(&packetUnion.stringPacket, byteArr, numBytes);
              Serial.println("El SS esta activo...");

              Serial.print("[SX1278] Message ID: ");
              Serial.println(packetUnion.stringPacket.messageID);
              Serial.print("[SX1278] Sender ID: ");
              Serial.println(packetUnion.stringPacket.senderID);
              Serial.print("[SX1278] Receiver ID: ");
              Serial.println(packetUnion.stringPacket.receiverID);

              if(int(packetUnion.stringPacket.messageID) == 255){
                Serial.println("Listo para enviar RTC...");
                transmitFlag = true; //Activo bandera de envio
                vTaskResume(xHandle_send_RTC_task);
              }
              break;
          case 2:
              memcpy(&packetUnion.thipacket, byteArr, numBytes);

              Serial.print("[SX1278] Message ID: ");
              Serial.println(packetUnion.thipacket.messageID);
              Serial.print("[SX1278] Sender ID: ");
              Serial.println(packetUnion.thipacket.senderID);
              Serial.print("[SX1278] Receiver ID: ");
              Serial.println(packetUnion.thipacket.receiverID);

              Serial.print(F("[SX1278] Payload:\t\t"));
              //Print the temperature and humidity
              Serial.print(F("Temperature: "));
              Serial.print(packetUnion.thipacket.temperature);
              Serial.print(F(" Humidity: "));
              Serial.println(packetUnion.thipacket.humidity);
              //Print the angle values
              Serial.print(F("Yaw: "));
              Serial.print(packetUnion.thipacket.yaw);
              Serial.print(F(" Pitch: "));
              Serial.print(packetUnion.thipacket.pitch);
              Serial.print(F(" Roll: "));
              Serial.println(packetUnion.thipacket.roll);


              if(packetUnion.thipacket.messageID == 200){
                if(xQueueSend(xQueueTempHumInc, &packetUnion.thipacket, portMAX_DELAY)){
                  Serial.println("Se envio la estructura THI a la cola");
                  vTaskResume(xHandle_send_mqtt_thi);
                }
                else{
                  Serial.println("No se pudo enviar la estructura bufferprueba a la cola xQueueBufferACL");
                }
              }
              break;
        }


      }
      else if (state == RADIOLIB_ERR_CRC_MISMATCH)
      {
        // packet was received, but is malformed
        Serial.println(F("[SX1278] CRC error!"));

        // EJECUTAR CASO ESPECIAL DE DATA CORRUPTA PARA AUMENTAR CURRENTPOS Y GUARDAR 0s EN BUFFER DE INTERES
        if (numBytes == 131)
        {
          fillBuffer(bufferactual, NULL, sizeof(packetUnion.packet1.payload), true);
        }
        else
        {
          Serial.println("[SX1278] La data recibida esta corrupta y no es para este receptor");
        }

        contador_errores++;
      }
      else if (state == -1)
      {
        // some other error occurred
        Serial.print(F("[SX1278] Failed, code "));
        Serial.println(state);
        contador_errores++;
      }
      else if (datacorrupta)
      {
        // some other error occurred
        Serial.print(F("[SX1278] Data corrupta..."));
        Serial.println(state);
        datacorrupta = false; ///Reinicio bandera
        contador_errores++;
      }

      Serial.print(F("[SX1278] Contador: "));
      Serial.println(contador);
      Serial.print(F("[SX1278] Contador error: "));
      Serial.println(contador_errores);

      Serial.println("");
    }
  else
  {
    Serial.println("Se estan enviando datos...");
  }
  vTaskSuspend(NULL);
  }
}

int sendmessage_radiolib(size_t size_data, byte data[])
{

  Serial.print(F("[SX1278] Transmitting packet ... "));

  // Crea una union de tipo packetunion para escoger el tipo de estructura de datos a utilizarse dependiendo del payload
  PacketUnion packetUnion;

  // Declaradas aqui para luego asignarles un valor en el if-else
  byte *packetBytes;
  size_t packetSize;

  if (size_data == 1)
  { // Es un comando
    packetUnion.packet2.messageID = 0x0;
    packetUnion.packet2.senderID = 0x1;                   // Set your sender ID
    packetUnion.packet2.receiverID = 0x2;                 // Set the intended receiver ID
    memcpy(packetUnion.packet2.payload, data, size_data); // Copy your byte array into the payload

    // Print the message payload and its length
    //  Serial.print(F("[SX1278] Payload:\t\t"));
    //  Serial.println(packetUnion.packet2.payload[0]);

    // Convert the Packet2 struct to a byte array
    byte *packetBytes = reinterpret_cast<byte *>(&packetUnion.packet2);
    packetSize = sizeof(packetUnion.packet2);
    // Send packetBytes using RadioLib
  }
  else
  {
    // Es actualizacion de RTC
    packetUnion.timePacket.messageID = 0xFF;                 // messageID designado para comandos
    packetUnion.timePacket.senderID = 0x1;                   // Set your sender ID
    packetUnion.timePacket.receiverID = 0x2;                 // Set the intended receiver ID
    memcpy(packetUnion.timePacket.payload, data, size_data); // Copy your byte array into the payload

    // Print the message payload and its length
    Serial.print(F("[SX1278] Payload:\t\t"));
    Serial.println(packetUnion.timePacket.payload[0]);

    // Convert the TimePacket struct to a byte array
    byte *packetBytes = reinterpret_cast<byte *>(&packetUnion.timePacket);
    packetSize = sizeof(packetUnion.timePacket);

    Serial.print("Length of packet to send: ");
    Serial.println(packetSize);
    // Send packetBytes using RadioLib
  }

  int state = radio.startTransmit(packetBytes, packetSize);

  if (state == RADIOLIB_ERR_NONE)
  {
    // the packet was successfully transmitted
    Serial.println(F(" success!"));
    return 1;
  }
  else if (state == RADIOLIB_ERR_PACKET_TOO_LONG)
  {
    // the supplied packet was longer than 256 bytes
    Serial.println(F("too long!"));
    return 0;
  }
  else if (state == RADIOLIB_ERR_TX_TIMEOUT)
  {
    // timeout occurred while transmitting packet
    Serial.println(F("timeout!"));
    return 0;
  }
  else
  {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);
    return 0;
  }
}

void update_timepacket(void)
{
  struct tm timeinfo = rtc.getTimeStruct();
  timestruct time_packet;
  time_packet.year = timeinfo.tm_year + 1900;
  time_packet.month = timeinfo.tm_mon + 1;
  time_packet.day = timeinfo.tm_mday;
  time_packet.hour = timeinfo.tm_hour;
  time_packet.minute = timeinfo.tm_min;
  time_packet.second = timeinfo.tm_sec;

  // Print the current time from the time_packet structure
  //  Print the current time from the time_packet structure
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

void send_RTC_task(void *pvParameters)
{
  while (1)
  {
    if (transmitFlag == true)
    {
      Serial.print(F("[SX1278] Transmitting RTC update... "));

      detachInterrupt(2);

      // LORA PACKET TO BE SENT
      TimePacket packet;

      // TIME PACKET STRUCTURE
      timestruct time_packet;
      // Fill the time_packet structure with current time
      struct tm timeinfo = rtc.getTimeStruct();

      time_packet.year = timeinfo.tm_year + 1900;
      time_packet.month = timeinfo.tm_mon + 1;
      time_packet.day = timeinfo.tm_mday;
      time_packet.hour = timeinfo.tm_hour;
      time_packet.minute = timeinfo.tm_min;
      time_packet.second = timeinfo.tm_sec;

      Serial.print("Year: ");
      Serial.print(time_packet.year);
      Serial.print(". Month: ");
      Serial.print(time_packet.month);
      Serial.print(". Day: ");
      Serial.print(time_packet.day);
      Serial.print(". Hour: ");
      Serial.print(time_packet.hour);
      Serial.print(". Minute: ");
      Serial.print(time_packet.minute);
      Serial.print(". Second: ");
      Serial.println(time_packet.second);

      // Convert the time_packet structure to a byte array
      byte *byteArrTime = (byte *)&time_packet;

      //Print the payload
      //print data of the packet
      Serial.print(F("[SX1278] Payload:\t\t"));
      //print the byte array
      // Print the byte array
      for (int i = 0; i < sizeof(time_packet); i++) {
          Serial.print(byteArrTime[i], HEX);
          Serial.print(F(" "));
      }


      Serial.println("");

      Serial.print(F("El tamaño del payload de tiempo es: "));
      Serial.println(sizeof(time_packet));

      packet.messageID = 255;
      packet.senderID = 1;                                         // Set your sender ID
      packet.receiverID = 2;                                       // Set the intended receiver ID
      memcpy(packet.payload, byteArrTime, sizeof(packet.payload)); // Copy the byte array into the payload

      // Convert the Packet struct to a byte array (ESTRUCTURA A ENVIAR POR LORA)
      byte *packetBytes = reinterpret_cast<byte *>(&packet);

      // Delay antes de transmitir para permitir que SS se ponga en modo listening
      delay(500);

      int state = radio.startTransmit(packetBytes, sizeof(packet));

      if (state == RADIOLIB_ERR_NONE)
      {
        // the packet was successfully transmitted
        Serial.println(F(" success!"));
      }
      else if (state == RADIOLIB_ERR_PACKET_TOO_LONG)
      {
        // the supplied packet was longer than 256 bytes
        Serial.println(F("muy largo!"));
      }
      else if (state == RADIOLIB_ERR_TX_TIMEOUT)
      {
        // timeout occurred while transmitting packet
        Serial.println(F("timeout!"));
      }
      else
      {
        // some other error occurred
        Serial.print(F("fallo, codigo "));
        Serial.println(state);
      }

      // Delay para permitir que se termine de enviar el paquete, no poner en modo receptor de inmediato
      delay(500);

      transmitFlag = false;

      Serial.print(F("[SX1278] Comienza a escuchar paquetes otra vez... \n"));

      int state2 = radio.startReceive();

      attachInterrupt(digitalPinToInterrupt(2), setFlag, RISING); // Reinicia ISR

      if (state2 == RADIOLIB_ERR_NONE)
      {
        Serial.println(F("Exito!"));
      }
      else
      {
        Serial.print(F("fallo, codigo "));
        Serial.println(state2);
        // while (true);
      }

      vTaskSuspend(NULL);
    }
    else
    {
      Serial.println("Se estan recibiendo datos en este momento...");
    }
  }
}

void send_task(void *pvParameters)
{
  while (1)
  {
    if (transmitFlag == true)
    {
      Serial.print(F("[SX1278] Transmitting packet ... "));

      detachInterrupt(2);

      byteArr[0] = 0x01;

      Packet2 packet;

      Serial.print(F("[SX1278] Payload size:\t\t"));
      Serial.println(sizeof(byteArr));

      packet.messageID = count++;
      packet.senderID = 1;                                     // Set your sender ID
      packet.receiverID = 2;                                   // Set the intended receiver ID
      memcpy(packet.payload, byteArr, sizeof(packet.payload)); // Copy your byte array into the payload

      // Convert the Packet struct to a byte array
      byte *packetBytes = reinterpret_cast<byte *>(&packet);

      int state = radio.startTransmit(packetBytes, sizeof(packet));

      if (state == RADIOLIB_ERR_NONE)
      {
        // the packet was successfully transmitted
        Serial.println(F(" success!"));
      }
      else if (state == RADIOLIB_ERR_PACKET_TOO_LONG)
      {
        // the supplied packet was longer than 256 bytes
        Serial.println(F("muy largo!"));
      }
      else if (state == RADIOLIB_ERR_TX_TIMEOUT)
      {
        // timeout occurred while transmitting packet
        Serial.println(F("timeout!"));
      }
      else
      {
        // some other error occurred
        Serial.print(F("fallo, coidgo "));
        Serial.println(state);
      }

      // Delay para permitir que se termine de enviar el paquete, no poner en modo receptor de inmediato
      delay(500);

      transmitFlag = false;

      Serial.print(F("[SX1278] Comienza a escuchar paquetes otra vez... \n"));

      update_timepacket();

      int state2 = radio.startReceive();

      attachInterrupt(digitalPinToInterrupt(2), setFlag, RISING); // Reinicia ISR

      if (state2 == RADIOLIB_ERR_NONE)
      {
        Serial.println(F("Exito!"));
      }
      else
      {
        Serial.print(F("fallo, codigo "));
        Serial.println(state2);
        // while (true);
      }

      vTaskSuspend(NULL);
    }
  }
}


void setup_lora(void)
{
  // initialize SX1278 with default settings
  Serial.print(F("[SX1278] Initializing ... "));
  int state = radio.begin();

  radio.setFrequency(433.0);
  radio.setSpreadingFactor(7);
  radio.setBandwidth(250.0);
  // radio.setOutputPower(15);

  // Set pin 2 as interrupt input pin
  pinMode(2, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), setFlag, RISING);

  if (state == RADIOLIB_ERR_NONE)
  {
    Serial.println(F("success!"));
  }
  else
  {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true)
      ;
  }

  // start listening for LoRa packets
  Serial.print(F("[SX1278] Starting to listen ... "));
  int state2 = radio.startReceive();

  if (state2 == RADIOLIB_ERR_NONE)
  {
    Serial.println(F("success!"));
  }
  else
  {
    Serial.print(F("failed, code "));
    Serial.println(state2);
    while (true)
      ;
  }
}
