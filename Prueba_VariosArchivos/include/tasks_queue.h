#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <Arduino.h>   

extern int counter;

// Declare the tasks
extern TaskHandle_t task1Handle;
extern TaskHandle_t task2Handle;
extern TaskHandle_t task3Handle;
extern TaskHandle_t task5Handle;
extern TaskHandle_t task4Handle;


// Task function prototypes
void task1(void* parameter);
void task2(void* parameter);
void task3(void* parameter);

void task4(void* parameter);
void task5(void* parameter);


// Queue handle
extern QueueHandle_t queue;

