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
#define F_SAMPLING 4

#define T_SAMPLING_TEMPHUM 200 //frecuencia = 1/T = 5Hz

#define NUM_DATOS 2048
#define NUM_DATOS_TEMP 50
// #define CS 5

//Sensor objects
Adafruit_BME280 bme;
Adafruit_MPU6050 mpu;

//Definicion de pines a utilizar
int ledpin= 26;

// Task handle

/*xTaskCreatePinnedToCore() creates a task and assigns a handle to xHandle_readBMETask. After this line, xHandle_readBMETask will no longer be NULL; it will hold a handle to the readBMETask task.

So, to answer your question, these variables could either be NULL (if they don't yet reference a task), or they could hold a handle to a task.*/
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
long int buffer_timestamp[NUM_DATOS - 1] = {};
long int buffer_timestamp_temp[NUM_DATOS_TEMP - 1] = {};
float bufferX[NUM_DATOS - 1] = { };
float bufferY[NUM_DATOS - 1] = { };
float bufferZ[NUM_DATOS - 1] = { };
float buffertemp[NUM_DATOS_TEMP/5 - 1] = { };
float bufferhum[NUM_DATOS_TEMP/5 - 1] = { };

int w = 0; //Contador para la cantidad de datos a guardar en el buffer de temperatura y humedad

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
        time_elapsed = (buffer_timestamp[NUM_DATOS - 1] - buffer_timestamp[2]);

        printf("Tiempo transcurrido: %d \n segundos", time_elapsed);
        //vTaskDelete(xHandle_crearBuffer);

        Serial.println("Suspendiendo tarea de recepcion de datos de temoeratura...");
        vTaskSuspend(xHandle_readBMETask); //Suspendo la tarea de recepcion de datos de temperatura

        Serial.println();
        Serial.println("Buffer de temperatura y humedad");
        for(int m = 0; m < 7; m++) {
          Serial.print("Temperatura: ");
          Serial.print(buffertemp[m]);
          Serial.print(" ");  
          Serial.print("Humedad: ");
          Serial.print(bufferhum[m]);
          Serial.println(" "); 
        }
        Serial.println("Me sali del loop");
        vTaskResume(xHandle_blink);
      }
    }
    else{
      Serial.println("No se recibio la cola correctamente...");
    };
 
    Serial.print("k: ");
    Serial.println(k);

    //Se suspende esta tarea para esperar la toma de datos
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


//Leer BME
void readBMETask(void *parameter) {
  while (true) {
    //Crea una estructura de tipo BMEData
    BMEData data_readtemp;

    //Lee los valores del sensor y los guarda en la estructura
    data_readtemp.humidity = bme.readHumidity();
    data_readtemp.temperature = bme.readTemperature();
    
    //Delay dependiendo del tiempo de muestreo
    vTaskDelay(T_SAMPLING_TEMPHUM / portTICK_PERIOD_MS); //5Hz

    //Envia los datos a la cola dataQueue
    if(xQueueSend(data_temphumQueue, &data_readtemp, portMAX_DELAY)){
      //Serial.println("Se envio correctamente la cola de temperatura");
      vTaskResume(xHandle_receive_temphum); //Reactivo tarea de creacion de buffers y promedios
    }
    else{
      Serial.println("No se envio correctamente la cola de temperatura");
    }

    // if(w <= NUM_DATOS_TEMP){
    //   vTaskResume(xHandle_receive_temphum);
    // }
  }
}

float prom_temp = 0;
float prom_hum = 0;
int cont = 1;
int cont2 = 0;
float buffer_prom[NUM_DATOS_TEMP/10];
// //Tarea de recepcion de datos
void receive_temphum(void *parameter) {
  while (true) {
    //Recibe la data de la cola y la guarda en una nueva estructura
    BMEData data_temphum;
    float valor_temp_actual;
    float valor_hum_actual;

    if(xQueueReceive(data_temphumQueue, &data_temphum, portMAX_DELAY)){
        //buffer_timestamp_temp[w] = millis();
        // buffertemp[w] = data_temphum.temperature;
        // bufferhum[w] = data_temphum.humidity;

    //CALCULO DEL PROMEDIO
        //Guardo las mediciones actuales
        valor_temp_actual = data_temphum.temperature;
        valor_hum_actual = data_temphum.humidity;
        //Las agrego al promedio
        prom_temp = valor_temp_actual + prom_temp;
        prom_hum = valor_hum_actual + prom_hum;
        // printf("El promedio actual de temperatura es: %f \n", prom_temp);
        // printf("El promedio actual de humedad es: %f \n", prom_temp);

        if(cont == 5){
          buffertemp[cont2] = prom_temp / 5;
          bufferhum[cont2] = prom_hum / 5;
          cont = 0; //reinicio el contador de valores obtenidos para sacar promedio
          cont2++; //Sumo 1  a este contador para guardar en siguiente posicion
          prom_temp = 0;
          prom_hum = 0; //Reinicio el promedio
        }
        cont++;
    }
    else{
      Serial.println("No se recibio la cola correctamente...");
    };
    
    vTaskSuspend(NULL); //Suspendo esta tarea hasta que se vuelva a activar desde otra

    // // //Recibo la cola y lo copio en la estructura data_temphum
    // if(xQueueReceive(data_temphumQueue, &data_temphum, portMAX_DELAY)){
    //   if(w < NUM_DATOS_TEMP){
    //     //Serial.println("Hola");
    //     buffer_timestamp_temp[w] = millis();
    //     buffertemp[w] = data_temphum.temperature;
    //     valor = data_temphum.temperature;
    //     prom = valor + prom;
    //     //printf("El promedio actual es: %f \n", prom);
    //     if(cont == 10){
    //       buffer_prom[cont2] = prom / 10;
    //       //printf("Se guardo nuevo valor en buffer promedio: %f \n", buffer_prom[cont2]);
    //       cont = 0; //reinicio el contador de valores obtenidos
    //       cont2++; //Sumo 1  a este contador para guardar en siguiente posicion
    //       prom = 0; //Reinicio el promedio
    //     }
    //     cont++;
    //     bufferhum[w] = data_temphum.humidity;
    //     w++; 
    //     //Serial.println(w);
    //   }
    //   else{
    //     Serial.println("Se termino de llenar el buffer de temperatura con exito!!!");
    //     printf("Valor final de w: %u \n", w);

    //     for(int m = 0; m < 5 ; m++) {
    //       Serial.print("Timestamp: ");
    //       Serial.print(buffer_timestamp_temp[m]);
    //       Serial.print("Temperatura: ");
    //       Serial.print(buffer_prom[m]);
    //       Serial.print(" ");  // Espacio entre los valores
    //       Serial.println(" ");  // Espacio entre los valores
    //     }
    //     for(int j = 0; j < 10; j++ ){
    //       Serial.print("Temperatura medida en los primeros 10: ");
    //       Serial.println(buffertemp[j]);
    //     }


    //     //Sacar promedio cada 50 valores y llenar nuevo buffer

    //     // Imprime los 20 valores en el puerto serial

    //     // for(int m = 0; m < 10 ; m++) {
    //     //   Serial.print("Timestamp: ");
    //     //   Serial.print(buffer_timestamp_temp[m]);
    //     //   Serial.print("Temperatura: ");
    //     //   Serial.print(buffertemp[m]);
    //     //   Serial.print(" ");  // Espacio entre los valores
    //     //   Serial.print("Humedad: ");
    //     //   Serial.print(bufferhum[m]);
    //     //   Serial.println(" ");  // Espacio entre los valores
    //     //    Serial.print(m);
    //     //   Serial.println(" ");  // Espacio entre los valores
    //     // }

    //     //Activa blink
    //     vTaskResume(xHandle_blink);
    //     //suspende esta tarea
    //     vTaskSuspend(xHandle_readBMETask);
    //     //suspende esta tarea
    //     vTaskSuspend(NULL);
    //   }
    // }
    // else{
    //   Serial.println("No se recibio la cola correctamente...");
    // };

  }
}

void blink(void * pvParameters ) {
    while (1) {
        digitalWrite(BUILTIN_LED, HIGH);
        Serial.println("Led encendido");
        delay(1000);
        digitalWrite(BUILTIN_LED, LOW);
        Serial.println("Led apagado");
        delay(1000);
    }
}

void setup() {

  //se configura puerto serial
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  //Creacion de las colas
  data_temphumQueue = xQueueCreate(2, sizeof(BMEData));
  aclQueue = xQueueCreate(3, sizeof(ACLData));
  bufferQueue = xQueueCreate(3, sizeof(ACLData));

  // Initialize the BME280 sensor
  while(!bme.begin(0x76)) {
    Serial.println("Could not find BME280 sensor!");
    while (1){
      delay(10);
    }
  }

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
    xTaskCreatePinnedToCore(readBMETask, "readBMETask", 1024*2, NULL, 0, &xHandle_readBMETask, 0);
    xTaskCreatePinnedToCore(receive_temphum, "receiveDataTask", 1024*2, NULL, 0, &xHandle_receive_temphum, 0);
    xTaskCreatePinnedToCore(blink, "readBMETask", 1024*2, NULL, 0, &xHandle_blink, 0);
    vTaskSuspend(xHandle_blink);
  
   
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