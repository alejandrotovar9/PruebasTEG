#include "tasks_queue.h"


//Create a task that receives a queue message every 2 seconds
void task4(void* parameter) {
  while (1) {
    //Receive a message from the queue
    int message;
    xQueueReceive(queue, &message, portMAX_DELAY);
    Serial.println("Received message from queue");
    vTaskDelay(pdMS_TO_TICKS(2000)); // Delay for 2 seconds
  }
}

// Task function for Task 5
void task5(void* parameter) {
  while (1) {
    //Toggle a led for 5 seconds
    digitalWrite(4, HIGH);
    vTaskDelay(pdMS_TO_TICKS(200)); // Delay for 1 second
    digitalWrite(4, LOW);
    vTaskDelay(pdMS_TO_TICKS(200)); // Delay for 1 second
  }
}
