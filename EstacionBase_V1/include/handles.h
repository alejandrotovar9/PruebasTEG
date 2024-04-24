#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <Arduino.h>

// //Handle de tareas de envio
TaskHandle_t xHandle_send_task = NULL;
TaskHandle_t xHandle_send_RTC_task = NULL;

//Handle para tareas de MQTT
TaskHandle_t xHandle_send_mqtt = NULL;
TaskHandle_t xHandle_keepalive_task= NULL;

// //Handle de tareas de recepcion
TaskHandle_t xHandle_receive_task = NULL;

//Handle de las colas del modulo de adquisicion inicializadas
QueueHandle_t xQueue;
QueueHandle_t xQueueBufferACL;
