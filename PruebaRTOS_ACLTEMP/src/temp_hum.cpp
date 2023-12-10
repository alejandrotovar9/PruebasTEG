#include <temp_hum.h>

extern TaskHandle_t xHandle_readBMETask;
extern TaskHandle_t xHandle_receiveDataTask;

extern QueueHandle_t dataQueue;

void temp_hum_setup(void){
// Initialize the BME280 sensor
  while(!bme.begin(0x76)) {
    Serial.println("Could not find BME280 sensor!");
    while (1){
      delay(10);
    }
  }
}

//Leer BME
void readBMETask(void *parameter){
  while (true) {
    //Crea una estructura de tipo BMEData
    BMEData data1;

    //Lee los valores del sensor y los guarda en la estructura
    data1.humidity = bme.readHumidity();
    data1.temperature = bme.readTemperature();
    
    //Envia los datos a la cola dataQueue
    xQueueSend(dataQueue, &data1, portMAX_DELAY);

    //Delay for 1 second
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

//Tarea de recepcion de datos
void receiveDataTask(void *parameter){
  while (true) {
    //Recibe la data de la cola y la guarda en una nueva estructura
    BMEData data;
    xQueueReceive(dataQueue, &data, portMAX_DELAY);

    // Print the data
    Serial.print("Temperatura: ");
    Serial.print(data.temperature);
    Serial.print(" *C\t");
    Serial.print("Humedad: ");
    Serial.print(data.humidity);
    Serial.println("%\t");
    // Serial.print("Presion: ");
    // Serial.println(data.pressure);
  }
}