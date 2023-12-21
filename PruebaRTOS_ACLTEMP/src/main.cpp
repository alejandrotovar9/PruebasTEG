#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <Wire.h>
#include <Arduino.h>
#include <accl.h>
#include <temp_hum.h>


//Creando objeto de clase BME280
Adafruit_BME280 bme;
Adafruit_MPU6050 mpu;

// //Variable global
// sensors_event_t a, g, tem;

//Intento de buffers
uint8_t buffer_timestamp[NUM_DATOS - 1];
float bufferX[NUM_DATOS - 1];
float bufferY[NUM_DATOS - 1];
float bufferZ[NUM_DATOS - 1];

//Handle de la Cola
QueueHandle_t xQueue;
QueueHandle_t aclQueue;
QueueHandle_t bufferQueue;
QueueHandle_t dataQueue;

//Handle de tareas
TaskHandle_t xHandle_crearBuffer;
TaskHandle_t xHandle_recibirDatosACL;
TaskHandle_t xHandle_leerDatosACL;
TaskHandle_t xHandle_blink;
TaskHandle_t xHandle_readBMETask;
TaskHandle_t xHandle_receiveDataTask;

void blink(void *pvParameters){
  pinMode(BUILTIN_LED, OUTPUT);
  while(1) {
      digitalWrite(BUILTIN_LED, 0);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      digitalWrite(BUILTIN_LED, 1);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}


void setup() {

  //Configuracion de puerto serial
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  //Creacion de las colas
  dataQueue = xQueueCreate(2, sizeof(BMEData));
  aclQueue = xQueueCreate(3, sizeof(ACLData));
  bufferQueue = xQueueCreate(3, sizeof(ACLData)); //No se esta usando aun

  //Inicializacion de sensores
  //temp_hum_setup();
  accl_setup();
   
  //Reloj del I2C 
  Wire.setClock(400000);
  Serial.println("Se ajusto el reloj.");

  //Creacion de tareas

    //Tareas a ejecutarse en el Nucleo 1 (lectura de hum y temp)
    //xTaskCreatePinnedToCore(readBMETask, "readBMETask", 1024*2, NULL, 1, &xHandle_readBMETask, 1);
    //xTaskCreatePinnedToCore(receiveDataTask, "receiveDataTask", 1024*2, NULL, 1, &xHandle_receiveDataTask, 1);
   
    //Tareas a ejecutarse en el Nucleo 0 (lectura de accl)
    xTaskCreatePinnedToCore(leerDatosACL, "leerDatosACL", 1024*2, NULL, 2, &xHandle_leerDatosACL, 0);
    xTaskCreatePinnedToCore(crearBuffer, "crearBuffer", 1024*2, NULL, 2, &xHandle_crearBuffer, 0);

    //Encender LED en nucleo 1
    xTaskCreatePinnedToCore(blink, "blink", 1024*2, NULL, 2, &xHandle_blink, 1);
    vTaskSuspend(xHandle_blink);
    Serial.println("PRUEBA");
}

void loop() {
  //No hacer nada
}
