#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <test1.h>
#include <test2.h>
#include "tasks_queue.h" //DECLARACION de handles como externs y prototipos de tareas
#include <handles.h> //Inicializacion de todos los handles

int counter = 15;

void setup()
{

  Serial.begin(115200);
  // Create a queue capable of containing 10 integers
  queue = xQueueCreate(10, sizeof(int));

  // Set pins as output
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);

  // Create Task 1
  // Create task pinned to core
  xTaskCreatePinnedToCore(
      task1,        /* Task function. */
      "Task1",      /* name of task. */
      10000,        /* Stack size of task */
      NULL,         /* parameter of the task */
      1,            /* priority of the task */
      &task1Handle, /* Task handle to keep track of created task */
      0);           /* pin task to core 0 */

  // Create Task 2
  xTaskCreatePinnedToCore(
      task2,        /* Task function. */
      "Task2",      /* name of task. */
      10000,        /* Stack size of task */
      NULL,         /* parameter of the task */
      1,            /* priority of the task */
      &task2Handle, /* Task handle to keep track of created task */
      0);           /* pin task to core 1 */

  // Create Task 3
  xTaskCreatePinnedToCore(
      task3,        /* Task function. */
      "Task3",      /* name of task. */
      10000,        /* Stack size of task */
      NULL,         /* parameter of the task */
      1,            /* priority of the task */
      &task3Handle, /* Task handle to keep track of created task */
      0);           /* pin task to core 0 */

  // Create Task 4
  xTaskCreatePinnedToCore(
      task4,        /* Task function. */
      "Task4",      /* name of task. */
      10000,        /* Stack size of task */
      NULL,         /* parameter of the task */
      1,            /* priority of the task */
      &task4Handle, /* Task handle to keep track of created task */
      0);           /* pin task to core 1 */

  xTaskCreatePinnedToCore(
      task5,        /* Task function. */
      "Task5",      /* name of task. */
      10000,        /* Stack size of task */
      NULL,         /* parameter of the task */
      1,            /* priority of the task */
      &task5Handle, /* Task handle to keep track of created task */
      0);           /* pin task to core 1 */

  vTaskSuspend(task5Handle);
}

void loop()
{
  // put your main code here, to run repeatedly:
}