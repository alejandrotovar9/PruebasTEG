#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <wifi_header.h>

#include <RadioLib.h>
#include <time.h>
#include <ESP32Time.h>

// //Handle de tareas de envio
extern TaskHandle_t xHandle_send_task;
extern TaskHandle_t xHandle_send_RTC_task;

// //Handle de tareas de recepcion
extern TaskHandle_t xHandle_receive_task;

//DECLARACION DE OBJETOS COMO EXTERNS PARA PODER USARLOS EN OTROS ARCHIVOS
extern ESP32Time rtc;