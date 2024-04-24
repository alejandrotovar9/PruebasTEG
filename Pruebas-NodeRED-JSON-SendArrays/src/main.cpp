#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <wifi_credentials.h>
#include <time.h>

#define PI 3.14159265

// WiFi and MQTT configuration
// const char* ssid = "your_wifi_ssid";
// const char* password = "your_wifi_password";
// const char* mqttBroker = "192.168.1.106";
const char* mqttBroker = "192.168.113.64";
//const char* mqttBroker = "192.168.20.64"; //TLF

const int mqttPort = 1883;
const char* mqttTopic1 = "esp32/prueba";

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -14400;
const int   daylightOffset_sec = 3600;

//HTTP Client
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Set your Static IP address
IPAddress local_IP(192, 168, 1, 184);
// Set your Gateway IP address
IPAddress gateway(192, 168, 1, 1);

IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(8, 8, 8, 8); // optional
IPAddress secondaryDNS(8, 8, 4, 4); // optional


//Creating a bigger array, with 500 elements
const size_t ARRAY_SIZE = 1024;
static float floatArrayX[ARRAY_SIZE];
static float floatArrayY[ARRAY_SIZE];
static float floatArrayZ[ARRAY_SIZE];

int frequency1 = 50;
int frequency2 = 100;
int frequency3 = 200;

//Function the fill the array with floats to form a sine wave
void fillArray(float* array, size_t size){
  for (size_t i = 0; i < size; i++) {
    array[i] = sin(2 * PI * frequency1 * i / size) + 0.7*sin(2 * PI * frequency2 * i / size) + 0.5*sin(2 * PI * frequency3 * i / size);
  }
}


void printLocalTime()
{
  struct tm timeinfo; //Estructura deberia ser global si se quiere acceder en otras tareas
  if(!getLocalTime(&timeinfo, 10000)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");

//Checking if the time is 5 PM
  if(timeinfo.tm_hour == 17){
    Serial.println("It's 5 PM");
  }

  //Updating ESP32 RTC time

}


// const size_t ARRAY_SIZE = 5;
// float floatArray[ARRAY_SIZE] = {1.23, 4.56, 7.89, 10.11, 12.13};

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  // Configures static IP address
  // if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
  //   Serial.println("STA Failed to configure");
  // }

  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  // Connect to WiFi
  // WiFi.begin(ssid, password);
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(1000);
  // }

  Serial.begin(115200);

  setup_wifi();

  //getrequest();

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer, "time.nist.gov", "time.google.com");
  printLocalTime();

  // Set up MQTT client
  mqttClient.setServer(mqttBroker, mqttPort);

  // Connect to MQTT broker
  while (!mqttClient.connected()) {
    if (mqttClient.connect("clientId")) {
      // Connected to MQTT broker
      Serial.println("Connected to MQTT broker...");
      break;
    } else {
      Serial.println("Error connecting to MQTT broker...");
      delay(1000);
    }
  }

  fillArray(floatArrayX, ARRAY_SIZE);
  fillArray(floatArrayY, ARRAY_SIZE);
  fillArray(floatArrayZ, ARRAY_SIZE);

  mqttClient.setBufferSize(50000);

  //Print the buffer size
  Serial.println("Buffer size: ");
  Serial.println(mqttClient.getBufferSize());

}

void reconnect() {
  // Loop until we're reconnected
  while (!wifiClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect("clientId")) {
      Serial.println("connected");

    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      vTaskDelay(5000/portTICK_PERIOD_MS);
    }
  }
}

void loop() {
  // Create JSON document to handle the 500 floats
  // DynamicJsonDocument doc(200);
  // //JsonDocument<200> doc;
  // JsonArray jsonArray = doc.to<JsonArray>();

  // // Add float array values to JSON array
  // for (size_t i = 0; i < ARRAY_SIZE; i++) {
  //   jsonArray.add(floatArray[i]);
  // }

  // // Serialize JSON to string
  // String jsonString;
  // serializeJson(jsonArray, jsonString);

  // Create a JSON document
  // StaticJsonDocument<200> doc;

  // Calculate the size of the JSON document
  // This should be large enough to hold the entire JSON object
  size_t jsonCapacity = JSON_ARRAY_SIZE(ARRAY_SIZE) * 3 + JSON_OBJECT_SIZE(3);

  // Create a DynamicJsonDocument with the calculated capacity
  DynamicJsonDocument doc(jsonCapacity);
  
// Add the float arrays to the document
JsonArray arrayX = doc.createNestedArray("x");
JsonArray arrayY = doc.createNestedArray("y");
JsonArray arrayZ = doc.createNestedArray("z");

for (size_t i = 0; i < ARRAY_SIZE; i++) {
  arrayX.add(floatArrayX[i]);
  arrayY.add(floatArrayY[i]);
  arrayZ.add(floatArrayZ[i]);
}
  
  // Serialize the document to a string
  String json;
  serializeJson(doc, json);

  //Print the size of the JSON Object in bytes
  Serial.println("Size of the JSON Object: ");
  Serial.println(json.length());

  if(mqttClient.publish(mqttTopic1, json.c_str())){
    Serial.println("Message published to MQTT topic");
  } else {
    Serial.println("Error publishing message to MQTT topic");
    if((!wifiClient.connected())){
      reconnect();
    }
  }

  //Print size of header
  //Serial.println("Size of the Header of the Object: ");

    // Publish JSON string to MQTT topic
  // if(mqttClient.publish(mqttTopic1, jsonString.c_str())){
  //   Serial.println("Message published to MQTT topic");
  // } else {
  //   Serial.println("Error publishing message to MQTT topic");
  //   if((!wifiClient.connected())){
  //     reconnect();
  //   }
  // }

  // Wait for some time before publishing again
  delay(5000);
}