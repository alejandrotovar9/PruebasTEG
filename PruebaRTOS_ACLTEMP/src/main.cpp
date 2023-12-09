#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <Wire.h>
#include <Arduino.h>

#define NUM_DATOS 5000

//Sensor objects
Adafruit_BME280 bme;
Adafruit_MPU6050 mpu;

//Definicion de pines a utilizar
int ledpin= 26;

// Task handle
TaskHandle_t xHandle_readBMETask = NULL;
TaskHandle_t xHandle_receiveDataTask = NULL;
TaskHandle_t xHandle_crearBuffer = NULL;
TaskHandle_t xHandle_recibirDatosACL = NULL;
TaskHandle_t xHandle_leerDatosACL = NULL;


//Handle de la Cola
QueueHandle_t xQueue;
QueueHandle_t dataQueue;
QueueHandle_t aclQueue;
QueueHandle_t bufferQueue;

//Variable global
sensors_event_t a, g, tem;
int num_datos = 5000;

// //Pines para I2C
// #define I2C_SDA  
// #define I2C_SCL  

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

      //Envia los datos a la cola dataQueue
      if(xQueueSend(aclQueue, &aclData, portMAX_DELAY)){
        Serial.println("Se envio la cola con datos...");
      }
      else{
        Serial.println("No se envio la cola...");
      }

      //Para ejecutar una sola vez mas para ver contenidos
      if(k<=5001){
        vTaskResume(xHandle_crearBuffer); //Reinicia la tarea para crear buffer
      }
  }
}

//Crear tarea para tomar cierta cantidad de datos (5000 por ej) y luego terminar el programa
//Para evitar el delay al llenar la SD mientras se adquieren los datos se puede usar un buffer temporal
//Luego de guardan los datos del buffer en SD y esto puede ejecutarse en el nucleo 2 del ESP
/*
Ejemplo:

Que tipo de datos es el que mas se ajusta?

struct BufferACL {
  array X;
  array Y;
  array Z;
};

Así o los lleno de ceros por el tamaño deseado (5000)
X = [];
Y = [];
Z = [];

Cuando recibo los datos los voy guardando en el buffer:
      BufferACL.X[k]= a.acceleration.x; 
      BufferACL.Y[k]= a.acceleration.y;
      BufferACL.Z[k]= a.acceleration.z;
      k++; aumento la posicion del vector en la cual voy a guardar el dato

Obtendriamos:
BufferACL.X = [1.05, 0.99, 0.95, 0.94, ...]
BufferACL.Y = [0.05, 0.07, 0.10, 0.09, ...]
BufferACL.Z = [0.50, 0.45, 0.55, 0.42, ...]

Luego en otra tarea accedo a esta estructura y guardo los datos en SD con el formato CSV deseado.
Ejecutando esta tarea en otro nucleo sin afectar el tiempo de muestreo.
Luego limpio el buffer y lo vuelvo a llenar y as sucesivamente.

Preguntas:
-Cual es el mejor tipo de datos a usar?
-Esto se guarda en RAM? Es decir, la estructura de datos a actualizarse, debe ser...
-Cuanto tiempo tarda esto en ejecutarse? El llenado de la estructura no debería tomar mucho tiempo, y 
el guardado en SD al estarse ejecutando en el otro nucleo no debería ser problema.
*/

//Estructura que contiene 3 arreglos de 5000 flotantes
struct BufferACL{
  float X[5000];
  float Y[5000];
  float Z[5000];
};

//Intento de buffers
uint8_t buffer_timestamp[5000] = {};
float bufferX[5000] = { };
float bufferY[5000] = { };
float bufferZ[5000] = { };

void crearBuffer(void *pvParameters){
  while(true){

    //Buffer a llenar
    //BufferACL buffer;
    ACLData datos_acl;

    //Recibo los datos de la cola y los guardo en la estructura creada
    if(xQueueReceive(aclQueue, &datos_acl, portMAX_DELAY)){
      if(k<=5000){
      buffer_timestamp[k] = millis();
      bufferX[k] = datos_acl.AclX;
      bufferY[k] = datos_acl.AclY;
      bufferZ[k] = datos_acl.AclZ;
      }
      else{
        Serial.println("Se termino de llenar la estructura con exito!!!");
        for(int w = 0; w < 20; w++){
          printf("Valores de tiempo: %u, X: %f \n", buffer_timestamp[w], bufferX[w]);
        }
        //vTaskDelete(xHandle_crearBuffer);
      }
    }
    else{
      Serial.println("No se recibio la cola correctamente...");
    };

    k++; 
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
  dataQueue = xQueueCreate(2, sizeof(BMEData));
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
    //xTaskCreatePinnedToCore(recibirDatosACL, "recibirDatosACL", 1024*2, NULL, 2, &xHandle_recibirDatosACL, 0);
}

void loop() {
  // Do nothing
}
