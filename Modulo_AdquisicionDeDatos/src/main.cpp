#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <Wire.h>
#include <Arduino.h>
#include <esp32-hal-log.h>
#include <SdFat.h>

#define NUM_DATOS 50
#define CS 5

//Sensor objects
Adafruit_BME280 bme;
Adafruit_MPU6050 mpu;

//Objetos de memoria SD
SdFat sd;
SdFile myFile;

// Task handle
TaskHandle_t xHandle_readBMETask = NULL;
TaskHandle_t xHandle_receiveDataTask = NULL;
TaskHandle_t xHandle_crearBuffer = NULL;
TaskHandle_t xHandle_recibirDatosACL = NULL;
TaskHandle_t xHandle_leerDatosACL = NULL;
TaskHandle_t xHandle_guardarSD = NULL;
TaskHandle_t xHandle_blink = NULL;

//Handle de la Cola
QueueHandle_t xQueue;
QueueHandle_t dataQueue;
QueueHandle_t aclQueue;
QueueHandle_t bufferQueue;

//Variable global
sensors_event_t a, g, tem;

//Buffers
uint8_t buffer_timestamp[NUM_DATOS - 1] = {};
float bufferX[NUM_DATOS - 1] = { };
float bufferY[NUM_DATOS - 1] = { };
float bufferZ[NUM_DATOS - 1] = { };

uint8_t timer1 = 0;
uint8_t timer2 = 0;
uint8_t tiempo = 0;

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

//Variables globales
int var = 0;
int k = 0; //Contador para la cantidad de datos a guardar en el array

//Leer BME
void readBMETask(void *parameter) {
  while (true) {
    //Crea una estructura de tipo BMEData
    BMEData data1;

    //Lee los valores del sensor y los guarda en la estructura
    data1.humidity = bme.readHumidity();
    data1.temperature = bme.readTemperature();
    
    //Envia los datos a la cola dataQueue
    xQueueSend(dataQueue, &data1, portMAX_DELAY);

    //Delay for 1 second
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void EnableInt(void *pvParameters){
      Wire.beginTransmission(0x68);  // Initialize the Tx buffer
      Wire.write(0x38);           // Put slave register address in Tx buffer
      Wire.write(0x01);                 // Put data in Tx buffer
      Wire.endTransmission();           // Send the Tx buffer
      //Interrupt activado
}

//MPU-6050 da los valores en enteros de 16 bits
//Valores RAW
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
//Ratios de conversion
#define A_R 16384.0 // 32768/2

// void leerDatosACL(void *pvParameters){
//   // /* Get new sensor events with the readings */
//      mpu.getEvent(&a, &g, &tem);
//   // }
//   //Leer los valores del Acelerometro de la IMU
//   while(true){

//       ACLData aclData; //Se define estructura a ser llenada

//       Wire.beginTransmission(0x68);
//       Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
//       Wire.endTransmission(false);
//       Wire.requestFrom(0x68 ,6, true);   //A partir del 0x3B, se piden 6 registros

//       //Se llena estructura con datos nuevos 
//       aclData.AclX = (Wire.read()<<8|Wire.read())/A_R; //Cada valor ocupa 2 registros
//       aclData.AclY = (Wire.read()<<8|Wire.read())/A_R;
//       aclData.AclZ = (Wire.read()<<8|Wire.read())/A_R;

//       //Envia los datos a la cola dataQueue
//       xQueueSend(aclQueue, &aclData, portMAX_DELAY);
//   }
// }


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


//Estructura que contiene 3 arreglos de 5000 flotantes
struct BufferACL{
  float X[NUM_DATOS - 1];
  float Y[NUM_DATOS - 1];
  float Z[NUM_DATOS - 1];
};

void crearBuffer(void *pvParameters){
  while(true){

    //Buffer a llenar
    //BufferACL buffer;
    ACLData datos_acl;

    //Recibo los datos de la cola y los guardo en la estructura creada
    if(xQueueReceive(aclQueue, &datos_acl, portMAX_DELAY)){
      if(k < NUM_DATOS){
        buffer_timestamp[k] = millis();
        bufferX[k] = datos_acl.AclX;
        bufferY[k] = datos_acl.AclY;
        bufferZ[k] = datos_acl.AclZ;
        k++; 
      }
      else{
        Serial.println("Se termino de llenar la estructura con exito!!!");
        printf("Valor final de k: %u \n", k);

        //Creando instancia de tipo BufferACL con el mismo nombre
        //BufferACL bufferACL;

        // Using memcpy to copy the arrays into the Data structure
        //memcpy(bufferACL.X, bufferX, sizeof(bufferX));
        // memcpy(bufferACL.Y, bufferY, sizeof(bufferY));
        // memcpy(bufferACL.Z, bufferZ, sizeof(bufferZ));

        // //Inicia el parpadeo del LED en el otro nucleo
        // vTaskResume(xHandle_blink);

        // Create a pointer to the structure
        //BufferACL* pData = &bufferACL;

        vTaskResume(xHandle_guardarSD);

        //se envia cola a otra tarea
        if(xQueueSend(bufferQueue, &bufferX, portMAX_DELAY) == pdTRUE){
          Serial.println("Se envio la cola a la tarea SD.");
        }
        else{
          Serial.println("Problema al enviar cola...");
        }
        // //vTaskDelete(xHandle_crearBuffer);
      }
    }
    else{
      Serial.println("No se recibio la cola correctamente...");
    };

    //Se suspende la tarea para esperar la toma de datos
    vTaskSuspend(xHandle_crearBuffer);
    //printf("BUFFER TASK: ESTOY AQUI TODAVIA \n");
  }
}


void guardarSD(void *pvParameters){
  while(true){

  float bufferXX[NUM_DATOS - 1] = { };  

  //BufferACL datos_listos;
  // Initialize SdFat or print a detailed error message and halt
  // Use half speed like the native library.
  // change to SPI_FULL_SPEED for more performance.
  if (!sd.begin(CS, SPI_HALF_SPEED)) sd.initErrorHalt();

  // open the file for write at end like the Native SD library
  if (!myFile.open("pruebaTEG1.txt", O_RDWR | O_CREAT | O_AT_END)) {
    sd.errorHalt("opening test.txt for write failed");
  }

  // if the file opened okay, write to it:

  //Headers
  myFile.println("ACLX");

  Serial.print("Se empieza a escribir en memoria...");
  if(xQueueReceive(bufferQueue, &bufferXX, portMAX_DELAY)){
     for(int w = 0; w < NUM_DATOS; w++){
      myFile.print(bufferXX[w]);
      myFile.print(",");
     }
  }
  //SUSPENDER TODO MIENTRAS SE GUARDA EN SD? O SE SIGUEN ADQUIRIENDO DATOS?

  // close the file:
  myFile.close();
  Serial.println("Se termino de guardar en SD.");
  }
}

void blink(void *pvParameters){
  pinMode(BUILTIN_LED, OUTPUT);
  while(1) {
      digitalWrite(BUILTIN_LED, 0);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      digitalWrite(BUILTIN_LED, 1);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
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

//Tarea de recepcion de datos
void receiveDataTask(void *parameter) {
  while (true) {
    //Recibe la data de la cola y la guarda en una nueva estructura
    BMEData data;
    xQueueReceive(dataQueue, &data, portMAX_DELAY);

    // Print the data
    Serial.print("Temperatura: ");
    Serial.print(data.temperature);
    Serial.print(" *C\t");
    Serial.print("Humedad: ");
    Serial.print(data.humidity);
    Serial.println("%\t");
    // Serial.print("Presion: ");
    // Serial.println(data.pressure);
  }
}

void setup() {

  //se configura puerto serial
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  //Creacion de las colas
  dataQueue = xQueueCreate(2, sizeof(BMEData));
  aclQueue = xQueueCreate(3, sizeof(ACLData));
  bufferQueue = xQueueCreate(3, sizeof(BufferACL));

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
    //xTaskCreatePinnedToCore(readBMETask, "readBMETask", 1024*2, NULL, 1, &xHandle_readBMETask, 1);
    //xTaskCreatePinnedToCore(receiveDataTask, "receiveDataTask", 1024*2, NULL, 1, &xHandle_receiveDataTask, 1);
   
    //Tareas a ejecutarse en el Nucleo 0
    xTaskCreatePinnedToCore(leerDatosACL, "leerDatosACL", 1024*2, NULL, 2, &xHandle_leerDatosACL, 0);
    xTaskCreatePinnedToCore(crearBuffer, "crearBuffer", 1024*2, NULL, 2, &xHandle_crearBuffer, 0);
    xTaskCreatePinnedToCore(guardarSD, "guardarSD", 1024*4, NULL, 2, &xHandle_guardarSD, 1);
    vTaskSuspend(xHandle_guardarSD);
    //Encender LED
    xTaskCreatePinnedToCore(blink, "blink", 1024*2, NULL, 2, &xHandle_blink, 1);
    vTaskSuspend(xHandle_blink);

    //xTaskCreatePinnedToCore(recibirDatosACL, "recibirDatosACL", 1024*2, NULL, 2, &xHandle_recibirDatosACL, 0);
}

void loop() {
  // Do nothing
}
