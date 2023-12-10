#include <Adafruit_BME280.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// //Handles de las tareas
// extern TaskHandle_t xHandle_readBMETask = NULL;
// extern TaskHandle_t xHandle_receiveDataTask = NULL;

// //Handle de la Cola
// extern QueueHandle_t dataQueue;

//Creando objeto de clase BME280
extern Adafruit_BME280 bme;

//Estructura de 2 flotantes para temp&hum
struct BMEData {
  float temperature;
  float humidity;
};

//Funciones
void temp_hum_setup(void);
void readBMETask(void *parameter);
void receiveDataTask(void *parameter);