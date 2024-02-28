#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <Wire.h>

#include <handles.h>
#include <tasks_queues.h>
#include <DAQ.h>


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

  //Creacion de las colas
  data_temphumQueue = xQueueCreate(1, sizeof(BMEData));
  aclQueue = xQueueCreate(3, sizeof(ACLData));
  bufferQueue = xQueueCreate(3, sizeof(ACLData));
  incQueue = xQueueCreate(1, sizeof(IncData));

  //Setting ouput pins for leds
  pinMode(LED_EST1, OUTPUT);
  pinMode(LED_IDLE, OUTPUT);
  pinMode(LED_CAL, OUTPUT);

  // Initialize the BME280 sensor

  /*Parametros iniciales del BME280 estan disponibles en el constructor del Adafruit_BME280.h
    void setSampling(sensor_mode mode = MODE_NORMAL,
                   sensor_sampling tempSampling = SAMPLING_X16,
                   sensor_sampling pressSampling = SAMPLING_X16,
                   sensor_sampling humSampling = SAMPLING_X16,
                   sensor_filter filter = FILTER_OFF,
                   standby_duration duration = STANDBY_MS_0_5);
  */

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

  Wire.setClock(400000); //Cambio en la frecuencia del reloj I2C

  //Creacion de tareas
    //Tareas a ejecutarse en el Nucleo 1
    xTaskCreatePinnedToCore(readBMETask, "readBMETask", 1024*2, NULL, 1, &xHandle_readBMETask, 1);
    xTaskCreatePinnedToCore(receive_temphum, "receiveDataTask", 1024*2, NULL, 1, &xHandle_receive_temphum, 1);
    vTaskSuspend(xHandle_receive_temphum);
    xTaskCreatePinnedToCore(blink, "readBMETask", 1024, NULL, 1, &xHandle_blink, 1);
    //vTaskSuspend(xHandle_blink);
  
    //Tareas a ejecutarse en el Nucleo 0
    xTaskCreatePinnedToCore(leerDatosACL, "leerDatosACL", 1024*2, NULL, 2, &xHandle_leerDatosACL, 0);
    xTaskCreatePinnedToCore(crearBuffer, "crearBuffer", 1024*2, NULL, 2, &xHandle_crearBuffer, 0);
    vTaskSuspend(xHandle_crearBuffer);

    //Tareas para obtener la inclinacion
    xTaskCreatePinnedToCore(readMPU9250, "readMPU9250", 1024*2, NULL, 0, &xHandle_readMPU9250, 1);
    xTaskCreatePinnedToCore(recInclinacion, "recInclinacion", 1024*2, NULL, 0, &xHandle_recInclinacion, 1);
    vTaskSuspend(xHandle_recInclinacion);
}

void loop() {
  delay(1000);
  // Do nothing
}
