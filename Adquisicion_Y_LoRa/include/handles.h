#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <Arduino.h>

//AQUI SE VAN A INICIALIZAR LAS QUEUES Y LOS HANDLES DE TAREAS

// Task handle inicializados para el modulo de adquisicion
TaskHandle_t xHandle_readBMETask = NULL;
TaskHandle_t xHandle_receive_temphum = NULL;

TaskHandle_t xHandle_leerDatosACL = NULL;
TaskHandle_t xHandle_crearBuffer = NULL;
TaskHandle_t xHandle_recibirDatosACL = NULL;

TaskHandle_t xHandle_readMPU9250 = NULL;
TaskHandle_t xHandle_recInclinacion= NULL;

TaskHandle_t xHandle_blink = NULL;

//Task handles para el modulo de comunicaciones LoRa
TaskHandle_t xHandle_poll_packet = NULL;
TaskHandle_t xHandle_send_packet = NULL;
TaskHandle_t xHandle_poll_modo_operacion = NULL;
TaskHandle_t xHandle_receiveTask = NULL;


//Handle de las colas del modulo de adquisicion inicializadas
QueueHandle_t xQueue;
QueueHandle_t data_temphumQueue;
QueueHandle_t aclQueue;
QueueHandle_t bufferQueue;
QueueHandle_t incQueue;
//Handles inicializados para modulo de comunicaciones LoRa
//Handle de la Cola
QueueHandle_t arrayQueue;
QueueHandle_t tramaLoRaQueue;