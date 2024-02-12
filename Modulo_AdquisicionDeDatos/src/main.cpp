#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <Wire.h>
#include <Arduino.h>

/*
Frecuencia de muestreo> f = 1/(0.001*F_SAMPLING)
1 - 1 KHz
2 - 500 Hz
3 - 333 Hz
4 - 250 Hz
5 - 200 Hz
6 - 166 Hz
7 - 142.85 Hz
*/
#define F_SAMPLING 3

#define NUM_DATOS 2048
#define NUM_DATOS_TEMP 10
// #define CS 5

//Sensor objects
Adafruit_BME280 bme;
Adafruit_MPU6050 mpu;

//Definicion de pines a utilizar
int ledpin= 26;

// Task handle
TaskHandle_t xHandle_readBMETask = NULL;
TaskHandle_t xHandle_receive_temphum = NULL;

TaskHandle_t xHandle_leerDatosACL = NULL;
TaskHandle_t xHandle_crearBuffer = NULL;
TaskHandle_t xHandle_recibirDatosACL = NULL;

TaskHandle_t xHandle_blink = NULL;


//Handle de la Cola
QueueHandle_t xQueue;
QueueHandle_t data_temphumQueue;
QueueHandle_t aclQueue;
QueueHandle_t bufferQueue;

//Variable global
sensors_event_t a, g, tem;

//Buffers
ushort buffer_timestamp[NUM_DATOS - 1] = {};
ushort buffer_timestamp_temp[NUM_DATOS_TEMP - 1] = {};

float bufferX[NUM_DATOS - 1] = { };
float bufferY[NUM_DATOS - 1] = { };
float bufferZ[NUM_DATOS - 1] = { };

int w = 0; //Contador para la cantidad de datos a guardar en el buffer de temperatura y humedad
float buffertemp[NUM_DATOS_TEMP - 1] = { };
float bufferhum[NUM_DATOS_TEMP - 1] = { };

// //Pines para I2C
// #define I2C_SDA  
// #define I2C_SCL  

//Estructura que contiene 3 arreglos de 5000 flotantes
struct BufferACL{
  float X[5000];
  float Y[5000];
  float Z[5000];
};

//Estructura de 2 flotantes para temp&hum
struct BMEData {
  float temperature;
  float humidity;
};

//Estructura de 3 flotantes para cada eje del accl
struct ACLData {
  float AclX;
  float AclY;
  float AclZ;
};

// //Estructura que contiene 3 arreglos de 5000 flotantes
// struct BufferACL{
//   float X[NUM_DATOS - 1];
//   float Y[NUM_DATOS - 1];
//   float Z[NUM_DATOS - 1];
// };


//Variables globales
int var = 0;
int k = 0; //Contador para la cantidad de datos a guardar en el array

//Leer BME
void readBMETask(void *parameter) {
  while (true) {
    //Crea una estructura de tipo BMEData
    BMEData data_readtemp;

    //Lee los valores del sensor y los guarda en la estructura
    data_readtemp.humidity = bme.readHumidity();
    data_readtemp.temperature = bme.readTemperature();
    
    //Envia los datos a la cola dataQueue
    if(xQueueSend(data_temphumQueue, &data_readtemp, portMAX_DELAY)){
      //Se envio la cola correctamente
      continue;
    }
    else{
      Serial.println("No se envio correctamente la cola de temperatura");
    }

    //Delay for 1 second
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    
//     if(k<= NUM_DATOS_TEMP){
//       vTaskResume(xHandle_receive_temphum);
//     }
  }
}

//Contador
int contador = 0;

void leerDatosACL(void *pvParameters){
  // /* Get new sensor events with the readings */

  //Leer los valores del Acelerometro de la IMU
  while(true){

      //EnableInt(); //Activamos interrupcion

      mpu.getEvent(&a, &g, &tem);

      ACLData aclData; //Estructura a ser llenada con 3 ejes

      //Se llena estructura con datos nuevos 
      aclData.AclX = a.acceleration.x; 
      aclData.AclY = a.acceleration.y;
      aclData.AclZ = a.acceleration.z;
      
      if(F_SAMPLING != 0){
        vTaskDelay(F_SAMPLING/portTICK_PERIOD_MS); //Reducir tiempo de muestreo
      }

      //Envia los datos a la cola dataQueue
      if(xQueueSend(aclQueue, &aclData, portMAX_DELAY)){
        //Serial.println("Se envio la cola con datos...");
        vTaskResume(xHandle_crearBuffer);
      }
      else{
        Serial.println("No se envio la cola...");
      }

      //vTaskResume(xHandle_crearBuffer); //Reinicia la tarea para crear buffer

      //Para ejecutar una sola vez mas para ver contenidos
      // if(k<=5001){
      //   vTaskResume(xHandle_crearBuffer); //Reinicia la tarea para crear buffer
      // }
  }
}

long int time_elapsed = 0;

void crearBuffer(void *pvParameters){
  while(true){

    //Buffer a llenar
    //BufferACL buffer;
    ACLData datos_acl;

    //Recibo los datos de la cola y los guardo en la estructura creada
    if(xQueueReceive(aclQueue, &datos_acl, portMAX_DELAY)){
      if(k<=NUM_DATOS){
      buffer_timestamp[k] = millis();
      bufferX[k] = datos_acl.AclX;
      bufferY[k] = datos_acl.AclY;
      bufferZ[k] = datos_acl.AclZ;
      k++;
      }
      else{
        Serial.println("Se termino de llenar la estructura con exito!!!");

        //AQUI SE INICIA LA TAREA PARA GUARDAR EN SD

        for(int w = 0; w < 30; w++){
          printf("Valores de tiempo: %u, X: %f \n", buffer_timestamp[w], bufferX[w]);
        }
        Serial.println(buffer_timestamp[NUM_DATOS - 1]);
        time_elapsed = (buffer_timestamp[NUM_DATOS - 1] - buffer_timestamp[2])/1000;

        printf("Tiempo transcurrido: %d \n segundos", time_elapsed);
        //vTaskDelete(xHandle_crearBuffer);
      }
    }
    else{
      Serial.println("No se recibio la cola correctamente...");
    };
 
    Serial.print("k: ");
    Serial.println(k);

    //Se suspende la tarea para esperar la toma de datos
    vTaskSuspend(xHandle_crearBuffer);

  }
}

//Recibir datos del acelerometro
void recibirDatosACL(void *pvParameters){
  while(true){
        // //Para graficar con teleplot
    ACLData aclData2;
    xQueueReceive(aclQueue, &aclData2, portMAX_DELAY);

    Serial.print(">Acceleration X:");
    Serial.println(aclData2.AclX);
    Serial.println(",");
    Serial.print(">Acceleration Y:");
    Serial.println(aclData2.AclY);
    // //Serial.print(",");
    // Serial.print(">Acceleration Z:");
    // Serial.println(aclData2.AclZ);
  }
}

// //Tarea de recepcion de datos
void receive_temphum(void *parameter) {
  while (true) {
    //Recibe la data de la cola y la guarda en una nueva estructura
    BMEData data_temphum;

    // if(xQueueReceive(data_temphumQueue, &data_temphum, portMAX_DELAY)){
    //     buffer_timestamp[w] = millis();
    //     buffertemp[w] = data_temphum.temperature;
    //     bufferhum[w] = data_temphum.humidity;
    //     w++; 
    // }
    // else{
    //   Serial.println("No se recibio la cola correctamente...");
    // };

    // //Recibo la cola y lo copio en la estructura data_temphum
    if(xQueueReceive(data_temphumQueue, &data_temphum, portMAX_DELAY)){
      if(w < NUM_DATOS_TEMP){
        buffer_timestamp_temp[w] = millis();
        buffertemp[w] = data_temphum.temperature;
        bufferhum[w] = data_temphum.humidity;
        w++; 
      }
      else{
        Serial.println("Se termino de llenar el buffer de temperatura con exito!!!");
        printf("Valor final de w: %u \n", w);

        // Imprime los 20 valores en el puerto serial
        for(int i = 0; i < NUM_DATOS_TEMP; i++) {
          Serial.print("Timestamp: ");
          Serial.print(buffer_timestamp_temp[i]);
          Serial.print("Temperatura: ");
          Serial.print(buffertemp[i]);
          Serial.print(" ");  // Espacio entre los valores
          Serial.print("Humedad: ");
          Serial.print(bufferhum[i]);
          Serial.println(" ");  // Espacio entre los valores
        }

        //Activa blink
        vTaskResume(xHandle_blink);
        //suspende esta tarea
        //vTaskDelete(xHandle_readBMETask);
        //suspende esta tarea
        //vTaskDelete(NULL);

        // //se envia cola a otra tarea con el buffer lleno de datos y timestamp
        // if(xQueueSend(bufferQueue, &bufferX, portMAX_DELAY) == pdTRUE){
        //   Serial.println("Se envio la cola a la tarea X.");
        // }
        // else{
        //   Serial.println("Problema al enviar cola...");
        // }
        // //vTaskDelete(xHandle_crearBuffer);
      }
    }
    else{
      Serial.println("No se recibio la cola correctamente...");
    };

     vTaskSuspend(NULL);
  }
}

// void wifiInit() {
//     Serial.print("Conectándose a ");
//     Serial.println(ssid);

//     WiFi.begin(ssid, password);

//     while (WiFi.status() != WL_CONNECTED) {
//       Serial.print(".");
//         vTaskDelay(500 / portTICK_PERIOD_MS);  
//     }
//     Serial.println("");
//     Serial.println("Conectado a WiFi");
//     Serial.println("Dirección IP: ");
//     Serial.println(WiFi.localIP());
//   }


void setup() {

  //se configura puerto serial
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  //Creacion de las colas
  data_temphumQueue = xQueueCreate(2, sizeof(BMEData));
  aclQueue = xQueueCreate(3, sizeof(ACLData));
  bufferQueue = xQueueCreate(3, sizeof(ACLData));

  // // Initialize the BME280 sensor
  // while(!bme.begin(0x76)) {
  //   Serial.println("Could not find BME280 sensor!");
  //   while (1){
  //     delay(10);
  //   }
  // }

  Serial.println("Adafruit MPU6050 test!");

  //Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  //float fmuestreo = 1/(F_SAMPLING*0.001);
  //printf("La frecuencia de muestreo escogida es: %f \n", fmuestreo);

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
   //Filtro Pasa Alto
  mpu.setHighPassFilter(MPU6050_HIGHPASS_DISABLE);
  //Introduce un retardo segun documentacion
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  mpu.setSampleRateDivisor(0);

  Wire.setClock(400000);

  //Creacion de tareas
    //Tareas a ejecutarse en el Nucleo 1
    //xTaskCreatePinnedToCore(readBMETask, "readBMETask", 1024*2, NULL, 0, &xHandle_readBMETask, 1);
    //xTaskCreatePinnedToCore(receive_temphum, "receiveDataTask", 1024*2, NULL, 0, &xHandle_receive_temphum, 1);
   //vTaskSuspend(xHandle_receive_temphum);
  
   
    //Tareas a ejecutarse en el Nucleo 0
    xTaskCreatePinnedToCore(leerDatosACL, "leerDatosACL", 1024*2, NULL, 2, &xHandle_leerDatosACL, 0);
    xTaskCreatePinnedToCore(crearBuffer, "crearBuffer", 1024*2, NULL, 2, &xHandle_crearBuffer, 0);
    //xTaskCreatePinnedToCore(recibirDatosACL, "recibirDatosACL", 1024*2, NULL, 2, &xHandle_recibirDatosACL, 0);
}

void loop() {
  // Do nothing
}


// void guardarSD(void *pvParameters){
//   while(true){

//   float bufferXX[NUM_DATOS - 1] = { };  

//   //BufferACL datos_listos;
//   // Initialize SdFat or print a detailed error message and halt
//   // Use half speed like the native library.
//   // change to SPI_FULL_SPEED for more performance.
//   if (!sd.begin(CS, SPI_HALF_SPEED)) sd.initErrorHalt();

//   // open the file for write at end like the Native SD library
//   if (!myFile.open("pruebaTEG1.txt", O_RDWR | O_CREAT | O_AT_END)) {
//     sd.errorHalt("opening test.txt for write failed");
//   }

//   // if the file opened okay, write to it:

//   //Headers
//   myFile.println("ACLX");

//   Serial.print("Se empieza a escribir en memoria...");
//   if(xQueueReceive(bufferQueue, &bufferXX, portMAX_DELAY)){
//      for(int w = 0; w < NUM_DATOS; w++){
//       myFile.print(bufferXX[w]);
//       myFile.print(",");
//      }
//   }
//   //SUSPENDER TODO MIENTRAS SE GUARDA EN SD? O SE SIGUEN ADQUIRIENDO DATOS?

//   // close the file:
//   myFile.close();
//   Serial.println("Se termino de guardar en SD.");
//   }
// }