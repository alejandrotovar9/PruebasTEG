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
int interval = 200;          // interval between sends

int NUM_PAQUETES_ESPERADOS = 32; //Numero de paquetes que se esperan enviar
int expected_length = 1; //Longitud esperada de los datos enviados por el receptor

//Contador de paquetes a enviar en funcion send_packet
int contador_paquetes = 0;

//Variables goblales para excepcion y conteo de errores
int error_count = 0;
int general_count = 0;


BufferACL trama;


int generararray(int contador_paquetes)
{
    //USO DE STATIC EVITA EL REBOOT
    static float floatArray[(SIZE_OF_FLOAT_ARRAY)]; //Creacion de float array

    //Recibiendo cola con datos de aceleracion en estructura de datos 
    //Los datos son un float array dentro del buffer
    if(contador_paquetes == 0){
          if(xQueueReceive(tramaLoRaQueue, &trama, portMAX_DELAY))
          {
          memcpy(floatArray, trama.bufferX, sizeof(trama.bufferX));
          //memcpy(floatArray + SIZE_OF_FLOAT_ARRAY, trama.bufferY, sizeof(trama.bufferY));
          // memcpy(floatArray + SIZE_OF_FLOAT_ARRAY * 2, trama.bufferZ, sizeof(trama.bufferZ));
          }
          //Serial.println("Se copiaron exitosamente los arreglos en FloatArray");
    }

    // Calculate the number of chunks
    int numChunks = sizeof(floatArray) / sizeof(float) / CHUNK_SIZE; //El numero de chunks indica el numero de bytes final
    //NUM_PAQUETES_ESPERADOS = numChunks;
    //Serial.print("Numero de chunks:");
    //Serial.println(numChunks);
    //printf("Numero de bytes total: %i", numChunks*4);
    
    // Create an array to hold the chunks
    float chunks[numChunks][CHUNK_SIZE];

    // Split the array into chunks
    for (int i = 0; i < numChunks; i++) {
      memcpy(chunks[i], &floatArray[i * CHUNK_SIZE], CHUNK_SIZE * sizeof(float)); //Copiar 128bytes de Float Array (a partir de la posicion especificada por i) en chunks
    }

    //se envia cola a otra tarea
    if(xQueueSend(arrayQueue, &chunks[contador_paquetes], portMAX_DELAY) == pdTRUE){
      Serial.println("Se envio la cola a la tarea de envio LoRa.");
    }
    else{
      Serial.println("Problema al enviar cola...");
      return 0;
    }

    return 1;
}

int leer_datos_sensor(int packetSize){

  if (packetSize == 0) return 0;          // if there's no packet, return 0

  Serial.println("Mensaje recibido");

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

  if(String(incoming[0], HEX) == "2"){
    //Se recibieron todos los paquetes en el receptor, no enviar mas paquetes
    Serial.println("Si es 2!!!");
    return 4;
  }
  else{
    //El paquete no se recibio con exito
    return 3;
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

int sendMessage(size_t size_data, byte data[]) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(size_data);      // add payload length
  //LoRa.print(outgoing);                 // add payload
  LoRa.write(data, size_data);
  LoRa.endPacket();                     // finish packet and send it
  if(msgCount < NUM_PAQUETES_ESPERADOS - 1){
    msgCount++;
  }
  else{
    msgCount = 0;
  }

  return 1;
}

//RUTINA DE POLLING PARA VERIFICAR EL MODO DE OPERACION DEL SENSOR INTELIGENTE
void poll_modo_operacion(void *pvParameters){
  while(1){
    int packetSize = LoRa.parsePacket();
    int modo_de_operacion = leer_modo_op(packetSize);

    switch(modo_de_operacion){
      case 0: 
      //Serial.println("No se recibio nada");
      break; //No se recibio ningun mensaje, seguir haciendo polling

      case 1: //Modo de operacion 1: Envio de trama de datos
      //Rutinas del modo de operacion 1
      Serial.println("Se selecciono el modo de operacion 1: Envio de trama de datos inmediata");
      //msgCount = 0;
      //Comienzo a crear registro de temperatura, humedad y aceleracion
      //Es decir, activo las banderas necesarias (flag_envio_inmediato) para enviar las colas
      vTaskResume(xHandle_send_packet); //Rutina de envio de trama de datos
      vTaskResume(xHandle_poll_packet); //Rutina de polling para mensajes del receptor'
      vTaskSuspend(NULL); //Suspendo temporalmente el polling de modo de operacion
      break;

      case 2: //Modo de operacion 2: Envio de datos continuos
      //Rutinas del modo de operacion 2
      Serial.println("Se selecciono el modo de operacion 2: Envio de datos continuos o ante una alarma de sobrepaso de limite en algun sensor");
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
        vTaskResume(xHandle_send_packet);
        general_count++;
        Serial.println("#####################################################");
        printf("Los errores para %d envios fueron: %d/%d \n", general_count, error_count, general_count);
        Serial.println();
        break;
        //vTaskResume(xHandle_send_packet); //La data llego con exito y tiene el formato deseado
      case 4:  //Respondo al emisor con que los datos ya fueron enviados por completo
        Serial.println("Se recibieron todos los paquetes en el receptor!!!");
        general_count = 0;
        error_count = 0;
        Serial.println("Desactivando rutina de envio de datos en el emisor...");
        vTaskSuspend(xHandle_send_packet);
        Serial.println("Reactivando rutina de lectura de modo de operacion del sensor...");
        //vTaskResume(xHandle_poll_modo_operacion);
        Serial.println("Desactivando esta rutina de polling recepcion de datos de sensores");
        vTaskSuspend(NULL); //Se suspende esta tarea
        break;
      case 5:
        Serial.println("Se recibio un paquete en el receptor pero no se reconoce el mensaje");
        break;
    }

    //vTaskDelay(10/portTICK_PERIOD_MS);
    //sx1278Interrupt = true; //reinicia flag de interrupcion
    digitalWrite(LED_CAL, LOW);
    vTaskSuspend(NULL); //Se suspende hasta la proxima interrupcion
    }
}

//PRUEBA
void send_packet(void *pvParameters){
  while(1){
    //Float array a enviar
     generararray(contador_paquetes); //Genero el array y mando un chunk, dependiendo del contador

     if(contador_paquetes < (SIZE_OF_FLOAT_ARRAY/NUM_PAQUETES_ESPERADOS) - 1){
         contador_paquetes++; //Aumento el contador para enviar el siguiente paquete en el proximo envio
     }
    else{
      contador_paquetes = 0; //Reinicio contador de paquetes
      }

     float floatarray[CHUNK_SIZE];
     byte data[CHUNK_SIZE * sizeof(float)]; //Inicializacion de byte array

     if(xQueueReceive(arrayQueue, &floatarray, portMAX_DELAY)){
        Serial.println("Se recibio la cola con los datos");
     }

    Serial.print("Contador de paquetes actual: ");
    Serial.println(contador_paquetes);
    // //Debo recibir los datos a enviar en una cola y formatearlos de ser necesario
    
    //Convert the float array to a byte array
     memcpy(data, floatarray, sizeof(floatarray)); //CHUNK_SIZE * sizeof(float)

    // Serial.print("Se recibio el siguiente chunk: ");
    // for(int w= 0; w < CHUNK_SIZE; w++){
    //       Serial.print(data[w], HEX); // Print each float
    //       Serial.print(" ");
    // }
    
    //printf("Tamaño de la cola: %d", sizeof(data));

    size_t size_data;
    /*byte data[]={0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02,0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05,
    0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01,
    0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01, 0x05, 0x04, 0x03, 0x02,0x01, 0x05, 0x04, 0x01, 0x05, 0x04, 0x03, 0x02,0x01, 0x05, 0x04};*/

    size_data = sizeof(data);

    //Llamo a la funcion que crea la trama de datos (payload)
    sendMessage(size_data, data);
    Serial.println("Sending byte array con datos!");
    Serial.println();

    //Espero X segundos luego de enviar mensaje
    vTaskDelay(interval/portTICK_PERIOD_MS);

    //Suspendo esta tarea hasta que se reciba otro mensaje
    // if(contador_paquetes >= SIZE_OF_FLOAT_ARRAY / NUM_PAQUETES_ESPERADOS){
    //   vTaskResume(xHandle_poll_modo_operacion);
    //   vTaskSuspend(NULL);
    // }
  }  
}

//RUTINA DE ENVIO DE DATOS DE SENSORES A ESTACION BASE
// void send_packet(void *pvParameters){
//   while(1)
//   {
//      generararray(contador_paquetes);

//     //  if(contador_paquetes < (SIZE_OF_FLOAT_ARRAY / NUM_PAQUETES_ESPERADOS) - 1){
//     //       contador_paquetes++; //Aumento el contador para enviar el siguiente paquete en el proximo envio
//     //       Serial.println("aumento paquete");
//     //  }
//     //  else
//     //  {
//     //   contador_paquetes = 0; //Reinicio contador de paquetes
//     //  }

//      Serial.println("prueba1");

//      float floatarray[CHUNK_SIZE]; //float array para recibir la cola
//      byte data[CHUNK_SIZE * sizeof(float)]; //Byte array a enviar

//      if(xQueueReceive(arrayQueue, &floatarray, portMAX_DELAY)){
//         Serial.println("Se recibio la cola con los datos");
//      }

//     //Convierte float array a bytearray
//      memcpy(data, floatarray, sizeof(floatarray)); //Copio en data sizeof(floatarray) = 128bytes de floatarray
//      //CHUNK_SIZE * sizeof(float)
//     Serial.println("prueba2");

//     // Serial.print("Se recibio el siguiente chunk: ");
//     // for(int w= 0; w < CHUNK_SIZE; w++){
//     //       Serial.print(data[w], HEX); // Print each float
//     //       Serial.print(" ");
//     // }
    
//     size_t size_data;
//     size_data = sizeof(data);
//     // Serial.print("Tamaño de la cola:");
//     // Serial.println(sizeof(data));

//     //Llamo a la funcion que crea la trama de datos (payload)
//     if(sendMessage(size_data, data)){
//         Serial.println("Sending byte array con datos!");
//         Serial.println();
//     }
//     else{
//       Serial.println("No se envio correctamente");
//     }

//     //Espero X segundos luego de enviar mensaje
//     vTaskDelay(interval/portTICK_PERIOD_MS);
//   }  
// }

void onReceive(int packetSize) {
  // received a packet
  //Serial.print("Received packet '");
  digitalWrite(LED_CAL, HIGH);
  vTaskResume(xHandle_poll_packet);
}


void setupLoRa(void){
     // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  if (!LoRa.begin(433E6)) {             // initialize ratio at 433 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }
  //LoRa.setCodingRate4(7);
  //LoRa.setPreambleLength(4);
  //LoRa.setSyncWord(0x12); //Establece la palabra de sincronizacion

  LoRa.setSignalBandwidth(125E3);//7.8E3 hasta 250E3, por defecto es 31.25E3
  LoRa.setSpreadingFactor(7);//entre 6 y 12
  //LoRa.enableCrc();

  //LoRa.receive();
  //LoRa_rxMode();

  //LoRa.onReceive(onReceive);

  Serial.println("LoRa init succeeded.");
}

