#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <MPU9250.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <Arduino.h>

//AQUI SE VAN A DECLARAR LAS TAREAS COMO PROTOTYPES Y LOS HANDLES COMO EXTERNS

// Task handles de Adquisicion de Datos
extern TaskHandle_t xHandle_readBMETask;
extern TaskHandle_t xHandle_receive_temphum;
extern TaskHandle_t xHandle_leerDatosACL;
extern TaskHandle_t xHandle_crearBuffer;
extern TaskHandle_t xHandle_recibirDatosACL;
extern TaskHandle_t xHandle_readMPU9250;
extern TaskHandle_t xHandle_recInclinacion;
extern TaskHandle_t xHandle_blink;

//Handle de la Colas
extern QueueHandle_t xQueue;
extern QueueHandle_t data_temphumQueue;
extern QueueHandle_t aclQueue;
extern QueueHandle_t bufferQueue;
extern QueueHandle_t incQueue;

//DECLARACION DE OBJETOS COMO EXTERNS PARA PODER USARLOS EN OTROS ARCHIVOS
extern Adafruit_BME280 bme;
extern Adafruit_MPU6050 mpu;
extern MPU9250 mpu9250;

//ESTA VARIABLE DE EVENTOS SOLO SE USA EN EL DAQ.CPP
extern sensors_event_t a, g, tem;