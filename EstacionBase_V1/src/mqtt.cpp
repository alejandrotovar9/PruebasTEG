#include <mqtt_header.h>
#include <lora_header.h>


//MQTT configuration
//const char* mqttBroker = "192.168.1.106";
const char* mqttBroker = "192.168.113.64";
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

//Function the fill the array with floats to form a sine wave
void fillArray(float* array, size_t size){
  for (size_t i = 0; i < size; i++) {
    array[i] = sin(2 * PI * 100 * i / size) + 0.7*sin(2 * PI * 200 * i / size) + 0.5*sin(2 * PI * 300 * i / size);
  }
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

void setup_mqtt() {

  // Set up MQTT client
  mqttClient.setServer(mqttBroker, mqttPort);

  //mqttClient.setKeepAlive(5000);

  // Connect to MQTT broker
  while (!mqttClient.connected()) {
    if (mqttClient.connect("clientId")) {
      // Connected to MQTT broker
      Serial.println("Connected to MQTT broker...");

      mqttClient.publish("test/topic", "Hello, World!");

      break;
    } else {
      Serial.println("Error connecting to MQTT broker...");
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

void send_mqtt(void *pvParameters){
    while(true){
        vTaskSuspend(xHandle_keepalive_task);

        //mqttClient.loop();

        //fillArray(floatArrayX, ARRAY_SIZE);
        
        //Receive float arrays from queue
        if(xQueueReceive(xQueueBufferACL, &bufferaceleracion, portMAX_DELAY)){
            Serial.println("Received from queue BufferACL");
        } else {
            Serial.println("Error receiving from queue");
        }

        delay(500);

        //mqttClient.publish("test/topic", "Hello, World from send mqtt!");

        // Calculate the size of the JSON document
        //size_t jsonCapacity = JSON_ARRAY_SIZE(ARRAY_SIZE) * 3 + JSON_OBJECT_SIZE(3);

        // // Create a DynamicJsonDocument with the calculated capacity
        // DynamicJsonDocument doc(40000);

    
        // // Add the float arrays to the document
        // JsonArray arrayX = doc.createNestedArray("x");
        // //JsonArray arrayY = doc.createNestedArray("y");
        // //JsonArray arrayZ = doc.createNestedArray("z");

        // //Print first 5 values from bufferX, bufferY and bufferZ
        // for (size_t i = 0; i < 5; i++) {
        //     Serial.print(bufferaceleracion.bufferX[i]);
        //     Serial.print(" ");
        //     Serial.print(bufferaceleracion.bufferY[i]);
        //     Serial.print(" ");
        //     Serial.print(bufferaceleracion.bufferZ[i]);
        //     Serial.println();
        // }

        // for (size_t i = 0; i < ARRAY_SIZE; i++) {
        //     arrayX.add(bufferaceleracion.bufferX[i]);
        //     //arrayY.add(bufferaceleracion.bufferY[i]);
        //     //arrayZ.add(bufferaceleracion.bufferZ[i]);
        // }
        
        //   // Serialize the document to a string
        // String json;


        // serializeJson(doc, json);

        // //Print the size of the JSON Object in bytes
        // Serial.println("Size of the JSON Object: ");
        // Serial.println(json.length());


        // if(mqttClient.publish(mqttTopic1, json.c_str()))
        // {
        //         Serial.println("Message published to MQTT topic");
        // } else {
        //     Serial.println("Error publishing message to MQTT topic");
        //     if((!wifiClient.connected())){
        //     reconnect();
        //     }
        // }

        sendAxis("esp32/x", "x", bufferaceleracion.bufferX, ARRAY_SIZE);
        sendAxis("esp32/y", "y", bufferaceleracion.bufferY, ARRAY_SIZE);
        sendAxis("esp32/z", "z", bufferaceleracion.bufferZ, ARRAY_SIZE);

        // Wait for some time before publishing again
        vTaskResume(xHandle_keepalive_task);
        vTaskSuspend(NULL);
    }
}
