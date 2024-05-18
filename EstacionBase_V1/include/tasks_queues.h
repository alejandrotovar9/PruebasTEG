#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <time.h>
#include <esp_task_wdt.h>

#include <wifi_header.h>

#include <RadioLib.h>
#include <ESP32Time.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// //Handle de tareas de envio
extern TaskHandle_t xHandle_send_task;
extern TaskHandle_t xHandle_send_RTC_task;

//Handle para tareas de MQTT
extern TaskHandle_t xHandle_send_mqtt;
extern TaskHandle_t xHandle_keepalive_task;
extern TaskHandle_t xHandle_send_mqtt_thi;

// //Handle de tareas de recepcion
extern TaskHandle_t xHandle_receive_task;

//Handle para Queue de envio de datos de buffer recibido via lora a ser enviado via MQTT
extern QueueHandle_t xQueueBufferACL;
extern QueueHandle_t xQueueTempHumInc;

//DECLARACION DE OBJETOS COMO EXTERNS PARA PODER USARLOS EN OTROS ARCHIVOS
extern ESP32Time rtc;