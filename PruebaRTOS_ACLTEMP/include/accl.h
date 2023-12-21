#include <Adafruit_MPU6050.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <Wire.h>

#define NUM_DATOS 5000

//Estructura de 3 flotantes para cada eje del accl
struct ACLData {
  float AclX;
  float AclY;
  float AclZ;
};

struct BufferACL {
  uint8_t buf_tstamp[NUM_DATOS - 1];
  float bufX[NUM_DATOS - 1];
  float bufY[NUM_DATOS - 1];
  float bufZ[NUM_DATOS - 1];
};

// // Task handle
// extern TaskHandle_t xHandle_crearBuffer = NULL;
// extern TaskHandle_t xHandle_recibirDatosACL = NULL;
// extern TaskHandle_t xHandle_leerDatosACL = NULL;
// extern TaskHandle_t xHandle_blink = NULL;

// //Handle de la Cola
// extern QueueHandle_t aclQueue;
// extern QueueHandle_t bufferQueue;

//Funciones
void leerDatosACL(void *pvParameters);
void crearBuffer(void *pvParameters);
void recibirDatosACL(void *pvParameters);
void accl_setup(void);