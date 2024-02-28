#include "tasks_queue.h"
#include <estructuras.h>

int c = 0;

BufferTempHumedad buffert;

// Task function for Task 1
void task1(void* parameter) {
  while (1) {
    //Toggle a led for 1 second
    digitalWrite(2, HIGH);
    vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    digitalWrite(2, LOW);
    vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    Serial.println(c);
    if(c >= counter){
        vTaskSuspend(task2Handle);
        vTaskResume(task5Handle);
    }
    c++;
  }
}

// Task function for Task 2
void task2(void* parameter) {
  while (1) {
    //Toggle another led for 0.5 second
    digitalWrite(4, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 0.5 second
    digitalWrite(4, LOW);
    vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 0.5 second
  }
}

//Create a task that sends a queue message every 1 second
void task3(void* parameter) {
  while (1) {
    //Send a message to the queue
    int message = 1;
    xQueueSend(queue, &message, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
  }
}
