

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <Wire.h>

#include <handles.h>
#include <tasks_queues.h>
#include <DAQ.h>
#include <lora_header.h>

//INICIALIZCION DE OBJETOS
Adafruit_BME280 bme;
Adafruit_MPU6050 mpu;
MPU9250 mpu9250;

//Variable de eventos MPU6050
sensors_event_t a, g, tem;

void setup() {
  //se configura puerto serial
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  //Creacion de las colas de DAQ
  data_temphumQueue = xQueueCreate(1, sizeof(BMEData));
  aclQueue = xQueueCreate(3, sizeof(ACLData));
  bufferQueue = xQueueCreate(3, sizeof(ACLData));
  incQueue = xQueueCreate(1, sizeof(IncData));
  tramaLoRaQueue = xQueueCreate(1, sizeof(BufferACL));

  //Creacion de las colas de Lora
  arrayQueue = xQueueCreate(1, CHUNK_SIZE * sizeof(float));
  if (arrayQueue == NULL) {
    // Failed to create the queue.
  }

  //Setting ouput pins for leds
  pinMode(LED_EST1, OUTPUT);
  pinMode(LED_IDLE, OUTPUT);
  pinMode(LED_CAL, OUTPUT);

  // Initialize the BME280 sensor
  while(!bme.begin(0x76)) {
    Serial.println("Could not find BME280 sensor!");
    while (1){
      delay(10);
    }
  }

  //Configuracion del acelerometro MPU6050
  setup_acl_MPU6050();
  //Configuracion del acelerometro 9DOF MPU9250
  setup_mpu9250();

  //Configuracion de modulo de comunicaciones LoRa
  setup_lora_radiolib();

  Wire.setClock(400000); //Cambio en la frecuencia del reloj I2C

  //Creacion de tareas

    //Tareas a ejecutarse en el Nucleo 0
    xTaskCreatePinnedToCore(leerDatosACL, "leerDatosACL", 1024*2, NULL, 2, &xHandle_leerDatosACL, 0);
    xTaskCreatePinnedToCore(crearBuffer, "crearBuffer", 1024*6, NULL, 2, &xHandle_crearBuffer, 0);
    vTaskSuspend(xHandle_crearBuffer);

    //Tareas a ejecutarse en el Nucleo 1
    xTaskCreatePinnedToCore(readBMETask, "readBMETask", 1024*2, NULL, 1, &xHandle_readBMETask, 1);
    xTaskCreatePinnedToCore(receive_temphum, "receiveDataTask", 1024*2, NULL, 1, &xHandle_receive_temphum, 1);
    vTaskSuspend(xHandle_receive_temphum);
    xTaskCreatePinnedToCore(blink, "readBMETask", 1024, NULL, 1, &xHandle_blink, 1);
    //vTaskSuspend(xHandle_blink);
  

    //Tareas para obtener la inclinacion Nucleo 1
    xTaskCreatePinnedToCore(readMPU9250, "readMPU9250", 1024*2, NULL, 0, &xHandle_readMPU9250, 1);
    xTaskCreatePinnedToCore(recInclinacion, "recInclinacion", 1024*2, NULL, 0, &xHandle_recInclinacion, 1);
    vTaskSuspend(xHandle_recInclinacion);

    //Tareas para la comunicacion LoRa Nucleo 1
    xTaskCreatePinnedToCore(send_packet, "send_packet", 1024*13, NULL, 2, &xHandle_send_packet, 0); 
    //Se modifico la memoria alojada para la tarea, antes era 1024*4
    vTaskSuspend(xHandle_send_packet);

    // xTaskCreatePinnedToCore(poll_packet, "poll_packet", 1024*2, NULL, 1, &xHandle_poll_packet, 0);
    // vTaskSuspend(xHandle_poll_packet);

    xTaskCreatePinnedToCore(receive_task, "receive_task", 1024*2, NULL, 3, &xHandle_receive_task, 1);
    vTaskSuspend(xHandle_receive_task);


    // xTaskCreatePinnedToCore(poll_modo_operacion, "poll_modo_op", 1024*2, NULL, 1, &xHandle_poll_modo_operacion, 0);
}

void loop() {
  delay(1000);
  // Do nothing
}
