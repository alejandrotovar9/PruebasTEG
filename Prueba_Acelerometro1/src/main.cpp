#include <Arduino.h>
#include <temp_hum.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

unsigned long tiempo = 0.0, tiempo1 = 0.0, tiempo2 = 0.0;
unsigned long contador = 0;
float  temp_acl_prom = 0.0, temp_acl = 0.0;
float raw_temp = 0.0, raw_hum = 0.0;
uint32_t Freq = 0;
const int CS = 5;
//const char path = "/test.txt";

File myFile;
Adafruit_MPU6050 mpu;

void WriteFile(const char * path, const char * message){
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open(path, FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile) {
    Serial.printf("Writing to %s ", path);
    myFile.println(message);
    myFile.close(); // close the file:
    Serial.println("completed.");
  } 
  // if the file didn't open, print an error:
  else {
    Serial.println("error opening file ");
    Serial.println(path);
  }
}


void ReadFile(const char * path){
  // open the file for reading:
  myFile = SD.open(path);
  if (myFile) {
     Serial.printf("Reading file from %s\n", path);
     // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    myFile.close(); // close the file:
  } 
  else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}

void setup(void) {
  Serial.begin(230400); //230400

  while (!Serial) { ; }  // wait for serial port to connect. Needed for native USB port only
  Serial.println("Initializing SD card...");
  if (!SD.begin(CS)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  //setup_temp();

  Wire.setClock(400000);

  mpu.setSampleRateDivisor(0);

  Serial.println(mpu.getCycleRate());

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  // MPU6050_RANGE_2_G = 0b00,  ///< +/- 2g (default value)
  // MPU6050_RANGE_4_G = 0b01,  ///< +/- 4g
  // MPU6050_RANGE_8_G = 0b10,  ///< +/- 8g
  // MPU6050_RANGE_16_G = 0b11, ///< +/- 16g
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  //Filtro Pasa Alto
  mpu.setHighPassFilter(MPU6050_HIGHPASS_1_25_HZ);

  //Introduce un retardo segun documentacion
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  /*Headers for CSV*/
  myFile = SD.open("/prueba2.txt", FILE_WRITE);
  if (myFile) {
    myFile.print("Time");
    myFile.print(",");
    myFile.print("ACLX");
    myFile.print(",");
    myFile.print("ACLY");
    myFile.print(",");
    myFile.println("ACLZ");
    myFile.close(); // close the file:
    Serial.println("Headers for CSV completed.");
  } 
  // if the file didn't open, print an error:
  else {
    Serial.println("error opening file ");
    Serial.println("/prueba2.txt");
  }
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

void EnableInt(){
      Wire.beginTransmission(0x68);  // Initialize the Tx buffer
      Wire.write(0x38);           // Put slave register address in Tx buffer
      Wire.write(0x01);                 // Put data in Tx buffer
      Wire.endTransmission();           // Send the Tx buffer
      //Interrupt activado
}

bool DataReadyInt(){
  uint8_t data; // `data` will store the register data
  Wire.beginTransmission(0x68);         // Initialize the Tx buffer
  Wire.write(0x3A);	                 // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(0x3A, (uint8_t) 1);  // Read one byte from slave register address
  data = Wire.read();                      // Fill Rx buffer with result
  //Serial.print(data);
  if(data & 0x01){
    return true;
  }
  else{
    return false;
  }
}
//Variables globales para acelerometro
sensors_event_t a, g, tem;

void tomarData(){
    /* Get new sensor events with the readings */
  // EnableInt();
  // if(DataReadyInt){
  //   mpu.getEvent(&a, &g, &tem);
  // }
  mpu.getEvent(&a, &g, &tem);
}

char bufferx[5000];

bool copiarCSVenSD(int num_datos){
  //myFile = SD.open("/prueba3.txt", FILE_WRITE);
  for (int i = 0; i < num_datos; i++) {
    myFile = SD.open("/prueba2.txt", FILE_WRITE);
    tomarData();
    bufferx[i] = a.acceleration.x;
    Serial.println(a.acceleration.x);
    myFile.print(bufferx[i]);
    myFile.close(); // close the file:
    // if (myFile) {
    // myFile.print(millis());
    // myFile.print(",");
    // myFile.print(a.acceleration.x);
    // Serial.println(a.acceleration.x);
    // myFile.print(",");
    // myFile.print(a.acceleration.y);
    // myFile.print(",");
    // myFile.print(a.acceleration.z);
    // myFile.println("");
    // myFile.close(); // close the file:
    // }
}
  //myFile.print(bufferx);
  //myFile.close(); // close the file:
  return true;
}

bool getCSV(int num_datos){
 int c = 0;
  for (int i = 0; i < num_datos; i++) {
    tomarData();
    Serial.print(millis());
    Serial.print(",");
    Serial.print(a.acceleration.x);
    Serial.print(",");
    // Serial.print(a.acceleration.y);
    // Serial.print(",");
    // Serial.print(a.acceleration.z);
    // Serial.print(",");
    //Modificar para que tome la primera vez y no sea 0 hasta la medicion 100
    // if(c = 100){
    //   //Al tomar estos datos se toma tiempo, por lo que los valores de aceleracion no son continuos
    //   //Hay un salto de aproximadamente .012 segundos cada vez
    //   getRawValues(&raw_temp, &raw_hum); //Actualizar valores de temp y hum
    //   Serial.print(raw_temp);
    //   Serial.print(",");
    //   Serial.print(raw_hum);
    //   Serial.print(",");
    //   c = 0;
    // }
    // else{
    //   Serial.print(raw_temp);
    //   Serial.print(",");
    //   Serial.print(raw_hum);
    //   Serial.print(",");
    // }
    // c++;
    Serial.println("");
}
  return true;
}

void plotAcl(){
    // //Para graficar con teleplot
  Serial.print(">Millis:");
  Serial.println(millis());
  Serial.print(">Acceleration X:");
  Serial.println(a.acceleration.x);
  //Serial.println(",");
  // Serial.print(">Acceleration Y:");
  // Serial.println(a.acceleration.y);
  // //Serial.print(",");
  // Serial.print(">Acceleration Z:");
  // Serial.println(a.acceleration.z);

  // //Para graficar valor de temperatura
  // Serial.print(">Temperatura del acelerometro:");
  // Serial.println(tem.temperature);

  // Serial.print(">Rotation X:");
  // Serial.println(g.gyro.x);
  // Serial.print(">Rotation Y:");
  // Serial.println(g.gyro.y);
  // Serial.print(">Rotation Z:");
  // Serial.println(g.gyro.z);
}

void plotTempAcl(){
  //Tomo promedio de temperatura del acelerometro
  temp_acl = temp_acl + tem.temperature;
  temp_acl_prom = temp_acl / contador;

    //Para graficar valor de temperatura
    Serial.print(">Temperatura del acelerometro:");
    Serial.println(temp_acl_prom);

    //Reiniciando variables
    temp_acl = 0.0;
    temp_acl_prom = 0;
}

//Arreglar
void plotTempHum(){

  getValues(); //Obtener valores de temperatura y humedad del BME280 para posteriormente promediar

  if(contador >= 100){
    printValues(); //Imprimir valores de temperatura y humedad promedio
    //Reiniciando contador
    contador = 0;
  }
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