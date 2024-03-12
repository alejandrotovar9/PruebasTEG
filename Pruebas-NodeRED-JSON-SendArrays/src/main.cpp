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
const char* mqttBroker = "192.168.1.102";
//const char* mqttBroker = "192.168.20.64"; //TLF

const int mqttPort = 1883;
const char* mqttTopic1 = "esp32/prueba";

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -14400;
const int   daylightOffset_sec = 3600;

//HTTP Client
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

//Function to make an HTTP Get Request to get the JSON for the local time
void getrequest(void){

    //Connect to HTTP server
  WiFiClient client;
  client.setTimeout(10000);
  if (!client.connect("http://worldtimeapi.org", 80)) {
    Serial.println(F("Connection failed"));
    return;
  }

  Serial.println(F("Connected!"));

  // Send HTTP request
  client.println(F("GET /api/timezone/America/Caracas HTTP/1.1"));
  client.println(F("Host: worldtimeapi.org"));
  client.println(F("Connection: close"));
  if (client.println() == 0) {
    Serial.println(F("Failed to send request"));
    client.stop();
    return;
  }

  // Check HTTP status
  char status[32] = {0};
  client.readBytesUntil('\r', status, sizeof(status));
  // It should be "HTTP/1.0 200 OK" or "HTTP/1.1 200 OK"
  if (strcmp(status + 9, "200 OK") != 0) {
    Serial.print(F("Unexpected response: "));
    Serial.println(status);
    client.stop();
    return;
  }

  // Skip HTTP headers
  char endOfHeaders[] = "\r\n\r\n";
  if (!client.find(endOfHeaders)) {
    Serial.println(F("Invalid response"));
    client.stop();
    return;
  }

  // Allocate the JSON document
  JsonDocument doc;

  // Parse JSON object
  DeserializationError error = deserializeJson(doc, client);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    client.stop();
    return;
  }

  // Extract values
  Serial.println(F("Response:"));
  String datetime = doc["datetime"].as<String>();
  Serial.println(datetime);

  String date = datetime.substring(0, datetime.indexOf('T'));
  Serial.println(date);

  // Disconnect
  client.stop();
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

//Creating a bigger array, with 500 elements
const size_t ARRAY_SIZE = 1024;
static float floatArray[ARRAY_SIZE];
int frequency1 = 50;
int frequency2 = 100;
int frequency3 = 200;

//Function the fill the array with floats to form a sine wave
void fillArray(float* array, size_t size){
  for (size_t i = 0; i < size; i++) {
    array[i] = sin(2 * PI * frequency1 * i / size) + 0.7*sin(2 * PI * frequency2 * i / size) + 0.5*sin(2 * PI * frequency3 * i / size);
  }
}

// const size_t ARRAY_SIZE = 5;
// float floatArray[ARRAY_SIZE] = {1.23, 4.56, 7.89, 10.11, 12.13};

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
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

  fillArray(floatArray, ARRAY_SIZE);

  // //Print the array created
  // for (size_t i = 0; i < ARRAY_SIZE; i++) {
  //   Serial.print(floatArray[i]);
  //   Serial.print(" ");
  // }
  // Serial.println();

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
  DynamicJsonDocument doc(200);
  //JsonDocument<200> doc;
  JsonArray jsonArray = doc.to<JsonArray>();

  // Add float array values to JSON array
  for (size_t i = 0; i < ARRAY_SIZE; i++) {
    jsonArray.add(floatArray[i]);
  }

  // Serialize JSON to string
  String jsonString;
  serializeJson(jsonArray, jsonString);

  //Print the size of the JSON Object in bytes
  Serial.println("Size of the JSON Object: ");
  Serial.println(jsonString.length());

  //Print size of header
  //Serial.println("Size of the Header of the Object: ");

    // Publish JSON string to MQTT topic
  if(mqttClient.publish(mqttTopic1, jsonString.c_str())){
    Serial.println("Message published to MQTT topic");
  } else {
    Serial.println("Error publishing message to MQTT topic");
    if((!wifiClient.connected())){
      reconnect();
    }
  }

  // Wait for some time before publishing again
  delay(5000);
}