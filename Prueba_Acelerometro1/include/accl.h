#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>

//Variables globales para acelerometro
sensors_event_t a, g, tem;

void setup_accl();
void tomarData();
bool DataReadyInt();
void EnableInt();
bool getCSV(int num_datos);
void plotAcl();
void plotTempAcl();