#include <Arduino.h>
#include <temp_hum.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

unsigned long tiempo = 0.0, tiempo1 = 0.0, tiempo2 = 0.0;
unsigned long contador = 0;
float  temp_acl_prom = 0.0, temp_acl = 0.0;
float raw_temp = 0.0, raw_hum = 0.0;
uint32_t Freq = 0;


Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(230400); //230400
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Freq = getCpuFrequencyMhz();
  // Serial.print("CPU Freq = ");
  // Serial.print(Freq);
  // Serial.println(" MHz");
  // Freq = getXtalFrequencyMhz();
  // Serial.print("XTAL Freq = ");
  // Serial.print(Freq);
  // Serial.println(" MHz");
  // Freq = getApbFrequency();
  // Serial.print("APB Freq = ");
  // Serial.print(Freq);
  // Serial.println(" Hz");

  setup_temp();

  Wire.setClock(400000);

  mpu.setSampleRateDivisor(0);

  Serial.println(mpu.getCycleRate());

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
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

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
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
  Serial.print("Time");
  Serial.print(",");
  Serial.print("ACLX");
  Serial.print(",");
  Serial.print("ACLY");
  Serial.print(",");
  Serial.print("ACLZ");
  Serial.print(",");
  Serial.print("Temp");
  Serial.print(",");
  Serial.println("Hum");

  // delay(100);
}

//Variables globales para acelerometro
sensors_event_t a, g, tem;

void tomarData(){
    /* Get new sensor events with the readings */
  mpu.getEvent(&a, &g, &tem);
}

bool getCSV(int num_datos){
 int c = 0;
  for (int i = 0; i < num_datos; i++) {
    tomarData();
    Serial.print(millis());
    Serial.print(",");
    Serial.print(a.acceleration.x);
    Serial.print(",");
    Serial.print(a.acceleration.y);
    Serial.print(",");
    Serial.print(a.acceleration.z);
    Serial.print(",");
    c++;
    //Modificar para que tome la primera vez y no sea 0 hasta la medicion 100
    if(c = 100){
      //Al tomar estos datos se toma tiempo, por lo que los valores de aceleracion no son continuos
      //Hay un salto de aproximadamente .012 segundos cada vez
      getRawValues(&raw_temp, &raw_hum); //Actualizar valores de temp y hum
      Serial.print(raw_temp);
      Serial.print(",");
      Serial.print(raw_hum);
      Serial.print(",");
      c = 0;
    }
    else{
      Serial.print(raw_temp);
      Serial.print(",");
      Serial.print(raw_hum);
      Serial.print(",");
    }
    Serial.println("");
}
  return true;
}

void plotAcl(){
    // //Para graficar con teleplot
  Serial.print(">Acceleration X:");
  Serial.println(a.acceleration.x);
  //Serial.println(",");
  Serial.print(">Acceleration Y:");
  Serial.println(a.acceleration.y);
  //Serial.print(",");
  Serial.print(">Acceleration Z:");
  Serial.println(a.acceleration.z);

  // Serial.print(">Rotation X:");
  // Serial.println(g.gyro.x);
  // Serial.print(">Rotation Y:");
  // Serial.println(g.gyro.y);
  // Serial.print(">Rotation Z:");
  // Serial.println(g.gyro.z);
}

void plotTempHum(){
  //Tomo promedio de temperatura del acelerometro
  temp_acl = temp_acl + tem.temperature;

  getValues(); //Obtener valores de temperatura y humedad del BME280 para posteriormente promediar

  if(contador >= 100){
    printValues(); //Imprimir valores de temperatura y humedad promedio

    temp_acl_prom = temp_acl / contador;

    //Para graficar valor de temperatura
    Serial.print(">Temperatura del acelerometro:");
    Serial.println(temp_acl_prom);

    //Reiniciando variables
    temp_acl = 0.0;
    temp_acl_prom = 0;
    //Reiniciando contador
    contador = 0;
  }
}

void loop() {

 //Obtener archivo CSV en puerto serial

  tiempo1 = millis();
 
  getCSV(5000);

  tiempo2 = millis();

  if(getCSV){
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
    Serial.println("Esperando reboot...");
    delay(1000);
  }


//----------------------GRAFICAR-------------------------------

  // tomarData();

  // //Graficar aceleraciones
  // plotAcl();

  // //Graficar temperatura y Humedad
  // plotTempHum();
 
  // //Aumento contador para el promedio de humedad y temperatura
  // contador++;

  //delay(500);
}