#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <Arduino.h>

// //Handle de tareas de envio
TaskHandle_t xHandle_send_task = NULL;
TaskHandle_t xHandle_send_RTC_task = NULL;

// //Handle de tareas de recepcion
TaskHandle_t xHandle_receive_task = NULL;