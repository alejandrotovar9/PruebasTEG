#include <Arduino.h>
#include <temp_hum.h>
#include <accl.h>
#include <saveSD.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

unsigned long tiempo = 0.0, tiempo1 = 0.0, tiempo2 = 0.0;
float raw_temp = 0.0, raw_hum = 0.0;
uint32_t Freq = 0;

void setup(void) {
  Serial.begin(230400); //230400

  setup_accl();
  setupSD();
  //setup_temp();

  Wire.setClock(400000);

  // Serial.print("Time");
  // Serial.print(",");
  // Serial.print("ACLX");
  // Serial.print(",");
  // Serial.print("ACLY");
  // Serial.print(",");
  // Serial.println("ACLZ");
  // Serial.print(",");
  // Serial.print("Temp");
  // Serial.print(",");
  // Serial.println("Hum");

  // delay(100);
}


//Arreglar
void plotTempHum(){

  getValues(); //Obtener valores de temperatura y humedad del BME280 para posteriormente promediar

  // if(contador >= 100){
  //   printValues(); //Imprimir valores de temperatura y humedad promedio
  //   //Reiniciando contador
  //   contador = 0;
  // }
}

void loop() {

//Obtener archivo CSV en Tarjeta SD

  tiempo1 = millis();
 
  copiarCSVenSD(5000);

  tiempo2 = millis();

    if(copiarCSVenSD){
    Serial.println("");
    Serial.println("Datos listos!");
    Serial.print("Tiempo:");
    tiempo = tiempo2 - tiempo1;
    Serial.println(tiempo);
  }
  else{
    Serial.print("Problema en la adquisicion!");
  }

  while(1){
    //Serial.println("Esperando reboot...");
    delay(1000);
  }

 //Obtener archivo CSV en puerto serial

  // tiempo1 = millis();
 
  // getCSV(5000);

  // tiempo2 = millis();

  // if(getCSV){
  //   Serial.println("");
  //   Serial.println("Datos listos!");
  //   Serial.print("Tiempo:");
  //   tiempo = tiempo2 - tiempo1;
  //   Serial.println(tiempo);
  // }
  // else{
  //   Serial.print("Problema en la adquisicion!");
  // }

  // while(1){
  //   //Serial.println("Esperando reboot...");
  //   delay(1000);
  // }


//----------------------GRAFICAR-------------------------------

  // tomarData();

  // //Graficar aceleraciones
  // plotAcl();

  // //Graficar temperatura y Humedad
  // //plotTempHum();
 
  // //Aumento contador para el promedio de humedad y temperatura
  // contador++;

  //delay(500);
}