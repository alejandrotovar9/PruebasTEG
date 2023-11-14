/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface. The device's I2C address is either 0x76 or 0x77.
 ***************************************************************************/
#include <temp_hum.h>

Adafruit_BME280 bme; // I2C

unsigned long delayTime;
float temp = 0.0;
float temp_prom = 0.0;
float hum = 0.0;
float hum_prom = 0.0;
int cont = 0;

void setup_temp() {
    Serial.begin(115200);
    while(!Serial);    // time to get serial running
    Serial.println(F("BME280 test"));

    unsigned status;
    
    // default settings
    status = bme.begin(0x76);  
    // You can also pass in a Wire library object like &Wire2
    // status = bme.begin(0x76, &Wire2)
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }
    
    Serial.println("-- Default Test --");
    delayTime = 100;

    Serial.println();
}

float getRawValues(){
    temp = bme.readTemperature();
    hum = bme.readHumidity();

    return temp,hum;
}

void getValues(){
    //Se leen valores de temperatura y humedad
    temp = temp + bme.readTemperature();
    hum = hum + bme.readHumidity();
    cont++;
}

void printValues() {
    //Imprimir y sacar promedio
    temp_prom = temp / cont;
    hum_prom = hum / cont;


    //Para graficar valor de temperatura y Humedad en serial plotter

    Serial.print(">Temperatura promedio:");
    //Serial.print(bme.readTemperature());
    Serial.println(temp_prom);
    //Serial.println(" °C");

    Serial.print(">Humedad promedio:");
    Serial.println(hum_prom);
    //Serial.println(" %");

    //Reiniciando variables
    temp = 0.0;
    hum = 0.0;
    temp_prom = 0.0;
    hum_prom = 0.0;
    cont = 0;
}