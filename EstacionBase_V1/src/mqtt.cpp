#include <mqtt_header.h>
#include <lora_header.h>


//MQTT configuration
//const char* mqttBroker = "192.168.1.106";
//const char* mqttBroker = "192.168.113.64";
const char* mqttBroker = "192.168.205.64";
//const char* mqttBroker = "192.168.20.64"; //TLF

const int mqttPort = 1883;
const char* mqttTopic1 = "esp32/prueba";

//HTTP Client
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// //Creating a bigger array, with 500 elements
const size_t ARRAY_SIZE = 1024;
//static float floatArrayX[ARRAY_SIZE];
// static float floatArrayY[ARRAY_SIZE];
// static float floatArrayZ[ARRAY_SIZE];

long lastReconnectAttempt = 0;

//Rutina de ISR por software en caso de recibir una peticion por MQTT
void IRAM_ATTR ISR_MQTT_Request()
{
  transmitFlag = true;
  vTaskResume(xHandle_send_task);
}


void reconnect() {
  // Loop until we're reconnected
  lastReconnectAttempt = 0;

  while (!wifiClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect("clientId")) {
      Serial.println("connected");
    }else{
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      vTaskDelay(5000/portTICK_PERIOD_MS);
    }
  }
}

void keepalive_task(void *pvParameters) {
    while (1) {
        // if (!mqttClient.connected()) {
        //     long now = millis();
        //     if (now - lastReconnectAttempt > 5000) {
        //         lastReconnectAttempt = now;
        //         // Attempt to reconnect
        //         reconnect();
        //     }
        // } else {

        mqttClient.publish("test/topic", "Hello, World!");
        mqttClient.loop();
        vTaskDelay(5000 / portTICK_PERIOD_MS); // Check every 5 seconds

    }
}

// Step 1: Define the callback function
void messageReceived(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received on topic: ");
  Serial.println(topic);

  // Step 2: Convert payload to string
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  // Step 3: Check if the message is "ON"
  if (message == "ON") {
    // Step 4: Call the software ISR
    Serial.println("Peticion de datos via MQTT... Enviando paquete Lora si no se estan enviando datos");
    if(!transmitFlag){
      ISR_MQTT_Request();
    }
  }
}

void setup_mqtt() {

  // Set up MQTT client
  mqttClient.setServer(mqttBroker, mqttPort);

  //mqttClient.setKeepAlive(5000);

  // Connect to MQTT broker
  while (!mqttClient.connected()) {
    if (mqttClient.connect("clientId")) {
      // Connected to MQTT broker
      Serial.println("Connected to MQTT broker...");

      digitalWrite(22, HIGH);

      mqttClient.publish("test/topic", "Hello, World!");

      // Step 2: Subscribe to a topic
      if(mqttClient.subscribe("esp32/command")){
        Serial.println("Suscrito a topico esp32/command");
      }
      else{
        Serial.println("Error al intentar suscribirse a topico esp32/command");
      }

      // Step 3: Set the callback function
      mqttClient.setCallback(messageReceived);

      break;
    } else {
      Serial.println("Error connecting to MQTT broker...");
      //mqttClient.setServer(mqttBroker2, mqttPort); //Intentando con otra IP que suele asignarse a la PC
      //Configurar numero de intentos de reconexion
      delay(1000);
    }
  }

      xTaskCreatePinnedToCore(
      keepalive_task,          /* Function to implement the task */
      "keepalive_task",        /* Name of the task */
      4096 * 1,              /* Stack size in words */
      NULL,                  /* Task input parameter */
      1,                     /* Priority of the task */
      &xHandle_keepalive_task, /* Task handle. */
      0);                    /* Core where the task should run */

  mqttClient.setBufferSize(40000);

  //mqttClient.setCallback(callback);

  //Print the buffer size
  Serial.println("Buffer size: ");
  Serial.println(mqttClient.getBufferSize());

  //vTaskResume(xHandle_keepalive_task);
}

//Creando estructura de tipo BufferACL para recibir cola
BufferACL bufferaceleracion;

// Define the maximum chunk size
const size_t CHUNK_SIZE_MQTT = 5000;

// Function to split a string into chunks and send each chunk as a separate message
void sendInChunks(const String& topic, const String& message) {
    for (size_t i = 0; i < message.length(); i += CHUNK_SIZE_MQTT) {

        // Calculate the number of chunks
        size_t numChunks = (message.length() + CHUNK_SIZE - 1) / CHUNK_SIZE;

        // Print the number of chunks
        Serial.println("Number of chunks: " + String(numChunks));
        // Get the next chunk of the message
        String chunk = message.substring(i, min(i + CHUNK_SIZE_MQTT, message.length()));

        // Send the chunk as a separate message
        mqttClient.publish((topic + String(i / CHUNK_SIZE_MQTT)).c_str(), chunk.c_str());

        // Wait for a short delay to avoid flooding the network
        delay(200);
    }
}

void sendTHI(const char* topic, int data){
    // // Convert the float to a string
    // char data_str[50];
    // dtostrf(data, 6, 2, data_str); // 6 is minimum width, 2 is precision; modify as needed

    // Convert the int to a string
    char data_str[50];
    itoa(data, data_str, 10); // 10 is the base for decimal numbers

    // Send the string
    if(mqttClient.publish(topic, data_str)) {
        Serial.println("Message published to MQTT topic");
    } else {
        Serial.println("Error publishing message to MQTT topic");
    }
}

void send_mqtt_thi(void *pvParameter){
  while(1){
    vTaskSuspend(xHandle_keepalive_task);

    THIPacket thipacket;

    //Receive float arrays from queue
    if(xQueueReceive(xQueueTempHumInc, &thipacket, portMAX_DELAY)){
        Serial.println("Received from queue TempInc");
    } else {
        Serial.println("Error receiving from queue");
    }

    delay(200);

    //FALTA EL TIMESTAMP OJO
    sendTHI("esp32/temp", thipacket.temperature);
    sendTHI("esp32/hum", thipacket.humidity);
    sendTHI("esp32/inc_y", thipacket.yaw);
    sendTHI("esp32/inc_p", thipacket.pitch);
    sendTHI("esp32/inc_r", thipacket.roll);
    sendTHI("esp32/timestamp", (int)thipacket.timestamp);

    vTaskResume(xHandle_send_mqtt);
    vTaskSuspend(NULL);
  }
}

void sendAxis(const char* topic, char* axis, const float* data, size_t dataSize) {
    // Create a DynamicJsonDocument
    DynamicJsonDocument doc(15000);

    // Add the float array to the document
    JsonArray array = doc.createNestedArray(axis);

    // Add data to the array
    for (size_t i = 0; i < dataSize; i++) {
        // Limit the float to 2 decimal places
        float value = round(data[i] * 100.0) / 100.0;
        array.add(value);
    }

    // // Add data to the array
    // for (size_t i = 0; i < dataSize; i++) {
    //     array.add(data[i]);
    // }

    // Serialize the document to a string
    String json;
    serializeJson(doc, json);

    
    //Print the size of the JSON Object in bytes
    Serial.println("Size of the JSON Object: ");
    Serial.println(json.length());

    // Send the JSON string in chunks
    //sendInChunks(topic, json);
    if(mqttClient.publish(topic, json.c_str()))
        {
            Serial.println("Message published to MQTT topic");
        } else {
            Serial.println("Error publishing message to MQTT topic");
            // if((!wifiClient.connected())){
            // reconnect();
            // }
        }
}

//Se activa una vez se reciben todos los paquetes Lora...
void send_mqtt(void *pvParameters){
    while(true){
        //vTaskSuspend(xHandle_keepalive_task);

        //mqttClient.loop();

        //fillArray(floatArrayX, ARRAY_SIZE);
        
        //Receive float arrays from queue
        if(xQueueReceive(xQueueBufferACL, &bufferaceleracion, portMAX_DELAY)){
            Serial.println("Received from queue BufferACL");
        } else {
            Serial.println("Error receiving from queue");
        }

        delay(500);

        sendAxis("esp32/x", "x", bufferaceleracion.bufferX, ARRAY_SIZE);
        sendAxis("esp32/y", "y", bufferaceleracion.bufferY, ARRAY_SIZE);
        sendAxis("esp32/z", "z", bufferaceleracion.bufferZ, ARRAY_SIZE);

        // Wait for some time before publishing again
        vTaskResume(xHandle_keepalive_task);
        vTaskSuspend(NULL);
    }
}
