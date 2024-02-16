#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <Wire.h>
#include <Arduino.h>

/*
Maximum delay time is 4.9 ms corresponding to just over 200 Hz sample rate
Frecuencia de muestreo> f = 1/(0.001*F_SAMPLING)
1 - 1 KHz
2 - 500 Hz
3 - 333 Hz
4 - 250 Hz
5 - 200 Hz
*/
#define F_SAMPLING 4
#define T_SAMPLING_TEMPHUM 200 //frecuencia = 1/T = 5Hz
#define NUM_DATOS 2048
#define MEAN_INTERVAL 5
#define NUM_DATOS_TEMP 200

//Sensor objects
Adafruit_BME280 bme;
Adafruit_MPU6050 mpu;

// Task handle
TaskHandle_t xHandle_readBMETask = NULL;
TaskHandle_t xHandle_receive_temphum = NULL;

TaskHandle_t xHandle_leerDatosACL = NULL;
TaskHandle_t xHandle_crearBuffer = NULL;
TaskHandle_t xHandle_recibirDatosACL = NULL;

TaskHandle_t xHandle_blink = NULL;


//Handle de la Cola
QueueHandle_t xQueue;
QueueHandle_t data_temphumQueue;
QueueHandle_t aclQueue;
QueueHandle_t bufferQueue;

//Variable global
sensors_event_t a, g, tem;

//Buffers
long int buffer_timestamp[NUM_DATOS - 1] = {};
long int buffer_timestamp_temp[NUM_DATOS_TEMP - 1] = {};
float bufferX[NUM_DATOS - 1] = { };
float bufferY[NUM_DATOS - 1] = { };
float bufferZ[NUM_DATOS - 1] = { };

//OJO ACOMODAR EL TAMANO DE ESTOS BUFFERS
float buffertemp[NUM_DATOS_TEMP - 1] = { };
float bufferhum[NUM_DATOS_TEMP - 1] = { };

//Estructuras de datos

struct BufferTempHumedad{
  float buffertemp[NUM_DATOS_TEMP/5 - 1] = { };
  float bufferhum[NUM_DATOS_TEMP/5 - 1] = { };
};
BufferTempHumedad estruc_buffer_datos;

int w = 0; //Contador para la cantidad de datos a guardar en el buffer de temperatura y humedad

//Estructura que contiene 3 arreglos de 5000 flotantes
struct BufferACL{
  long int buffer_timestamp[NUM_DATOS - 1] = {};
  float bufferX[NUM_DATOS - 1] = { };
  float bufferY[NUM_DATOS - 1] = { };
  float bufferZ[NUM_DATOS - 1] = { };
};

//Estructura de 2 flotantes para temp&hum
struct BMEData {
  float temperature;
  float humidity;
};

//Estructura de 3 flotantes para cada eje del accl
struct ACLData {
  float AclX;
  float AclY;
  float AclZ;
};

// //Estructura que contiene 3 arreglos de 5000 flotantes
// struct BufferACL{
//   float X[NUM_DATOS - 1];
//   float Y[NUM_DATOS - 1];
//   float Z[NUM_DATOS - 1];
// };

//Variables globales
int var = 0;
int k = 0; //Contador para la cantidad de datos a guardar en el array

void leerDatosACL(void *pvParameters){
  // /* Get new sensor events with the readings */

  //Leer los valores del Acelerometro de la IMU
  while(true){

      //EnableInt(); //Activamos interrupcion

      mpu.getEvent(&a, &g, &tem);

      ACLData aclData; //Estructura a ser llenada con 3 ejes

      //Se llena estructura con datos nuevos 
      aclData.AclX = a.acceleration.x; 
      aclData.AclY = a.acceleration.y;
      aclData.AclZ = a.acceleration.z;
      
      if(F_SAMPLING != 0){
        vTaskDelay(F_SAMPLING/portTICK_PERIOD_MS); //Reducir tiempo de muestreo
      }

      //Envia los datos a la cola dataQueue
      if(xQueueSend(aclQueue, &aclData, portMAX_DELAY)){
        //Serial.println("Se envio la cola con datos...");
        vTaskResume(xHandle_crearBuffer);
      }
      else{
        Serial.println("No se envio la cola...");
      }

      //vTaskResume(xHandle_crearBuffer); //Reinicia la tarea para crear buffer
  }
}

long int time_elapsed = 0;
long int tiempo1 = 0;
long int tiempo2 = 0;

//Solo funciona la estructura de aceleracion si la declaro como global
//Si la declaro dentro de la tarea no guarda los datos en los buffers respectivos
BufferACL struct_buffer_acl;

void crearBuffer(void *pvParameters){
  while(true){

    //Buffer a llenar
    //BufferACL buffer;
    ACLData datos_acl;

    //Recibo los datos de la cola y los guardo en la estructura creada
    if(xQueueReceive(aclQueue, &datos_acl, portMAX_DELAY)){
      if( k <= NUM_DATOS){
        if(k == 1) tiempo1 = millis();
        struct_buffer_acl.buffer_timestamp[k] = millis();
        struct_buffer_acl.bufferX[k] = datos_acl.AclX;
        struct_buffer_acl.bufferY[k] = datos_acl.AclY;
        struct_buffer_acl.bufferZ[k] = datos_acl.AclZ;
        // bufferX[k] = datos_acl.AclX;
        // bufferY[k] = datos_acl.AclY;
        // bufferZ[k] = datos_acl.AclZ;
        k++;
      }
      else{
        Serial.println("Se termino de llenar la estructura con exito!!!");
        tiempo2 = millis();

        for(int w = 0; w < 30; w++){
          printf("Valores de tiempo: %u, X: %f Y: %f Z: %f \n", struct_buffer_acl.buffer_timestamp[w], struct_buffer_acl.bufferX[w], struct_buffer_acl.bufferY[w], struct_buffer_acl.bufferZ[w]);
        }

        time_elapsed = (tiempo2 - tiempo1);
        printf("Tiempo transcurrido: %d segundos \n", time_elapsed / 1000.0);

        Serial.println("Suspendiendo tarea de recepcion de datos de temperatura...");
        vTaskSuspend(xHandle_readBMETask); //Suspendo la tarea de recepcion de datos de temperatura

        Serial.println();
        Serial.println("Buffer de temperatura y humedad");
        for(int m = 0; m < 8; m++) {
          Serial.print("Temperatura: ");
          Serial.print(estruc_buffer_datos.buffertemp[m]);
          Serial.print(" ");  
          Serial.print("Humedad: ");
          Serial.print(estruc_buffer_datos.bufferhum[m]);
          Serial.println(" "); 
        }
        vTaskResume(xHandle_blink);
      }
    }
    else{
      Serial.println("No se recibio la cola correctamente...");
    };
 
    // Serial.print("k: ");
    // Serial.println(k);

    //Se suspende esta tarea para esperar la toma de datos
    vTaskSuspend(xHandle_crearBuffer);

  }
}

//Recibir datos del acelerometro
void recibirDatosACL(void *pvParameters){
  while(true){
        // //Para graficar con teleplot
    ACLData aclData2;
    xQueueReceive(aclQueue, &aclData2, portMAX_DELAY);

    Serial.print(">Acceleration X:");
    Serial.println(aclData2.AclX);
    Serial.println(",");
    Serial.print(">Acceleration Y:");
    Serial.println(aclData2.AclY);
    // //Serial.print(",");
    // Serial.print(">Acceleration Z:");
    // Serial.println(aclData2.AclZ);
  }
}

//Leer BME
void readBMETask(void *parameter) {
  while (true) {
    //Crea una estructura de tipo BMEData
    BMEData data_readtemp;

    //Lee los valores del sensor y los guarda en la estructura
    data_readtemp.humidity = bme.readHumidity();
    data_readtemp.temperature = bme.readTemperature();

    //Envia los datos a la cola dataQueue
    if(xQueueSend(data_temphumQueue, &data_readtemp, portMAX_DELAY)){
      //Serial.println("Se envio correctamente la cola de temperatura");
      vTaskDelay(10 / portTICK_PERIOD_MS); //Este delay permite que las lecturas del sensor sean las correctas
      vTaskResume(xHandle_receive_temphum); //Reactivo tarea de creacion de buffers y promedios
    }
    else{
      Serial.println("No se envio correctamente la cola de temperatura");
    }

    //Delay dependiendo del tiempo de muestreo
    vTaskDelay(T_SAMPLING_TEMPHUM / portTICK_PERIOD_MS); //5Hz

    // if(w <= NUM_DATOS_TEMP){
    //   vTaskResume(xHandle_receive_temphum);
    // }
  }
}

float prom_temp = 0;
float prom_hum = 0;
int cont = 1;
int cont2 = 0;

// //Tarea de recepcion de datos
void receive_temphum(void *parameter) {
  while (true) {
    //Recibe la data de la cola y la guarda en una nueva estructura
    BMEData data_temphum;
    
    float valor_temp_actual = 0;
    float valor_hum_actual = 0;

    if(xQueueReceive(data_temphumQueue, &data_temphum, portMAX_DELAY)){

    //CALCULO DEL PROMEDIO
        //Guardo las mediciones actuales
        valor_temp_actual = data_temphum.temperature;
        valor_hum_actual = data_temphum.humidity;
        //Las agrego al promedio
        prom_temp = valor_temp_actual + prom_temp;
        prom_hum = valor_hum_actual + prom_hum;

        if(cont >= MEAN_INTERVAL){
         vTaskDelay(1 / portTICK_PERIOD_MS);
         //SI NO IMPRIMO ESTO EN SERIAL SACA MAL EL PROMEDIO... PROBLEMAS DE VELOCIDAD?
          printf("El  actual de temperatura es: %f \n", valor_temp_actual);
          printf("El  actual de humedad es: %f \n", valor_hum_actual);
          estruc_buffer_datos.buffertemp[cont2] = prom_temp / cont;
          estruc_buffer_datos.bufferhum[cont2] = prom_hum / cont;

          cont = 0; //reinicio el contador de valores obtenidos para sacar promedio
          cont2++; //Sumo 1  a este contador para guardar en siguiente posicion
          prom_temp = 0;
          prom_hum = 0; //Reinicio el promedio
        }
        cont++;
    }
    else{
      Serial.println("No se recibio la cola correctamente...");
    };
    
    vTaskSuspend(NULL); //Suspendo esta tarea hasta que se vuelva a activar desde otra
  }
}

void blink(void * pvParameters ) {
    while (1) {
        digitalWrite(BUILTIN_LED, HIGH);
        Serial.println("Led encendido");
        delay(1000);
        digitalWrite(BUILTIN_LED, LOW);
        Serial.println("Led apagado");
        delay(1000);
    }
}

void setup_acl_MPU6050(void){

  int16_t accelCount[3];           // Stores the 16-bit signed accelerometer sensor output
  float gyroBias[3], accelBias[3]; // Bias corrections for gyro and accelerometer
  float SelfTest[6];               // Gyro and accelerometer self-test sensor output
  uint32_t count = 0;
  
  Serial.println("Inicializacion del Adafruit MPU6050!");

  //Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Fallo al encontrar el MPU6050");
    while (1) {
      delay(10);
    }
  }

  Serial.println("MPU6050 encontrado!");
  Serial.println("MPU6050");
  Serial.println("6-DOF 16-bit");
  Serial.println("motion sensor");
  Serial.println("60 ug LSB");
  //float fmuestreo = 1/(F_SAMPLING*0.001);
  //printf("La frecuencia de muestreo escogida es: %f \n", fmuestreo);

  // mpu.MPU6050SelfTest(SelfTest); // Start by performing self test and reporting values
  //   Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
  //   Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
  //   Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
  //   Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
  //   Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
  //   Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");

  //   /*EL BIAS HABRIA QUE RESTARSELO A LOS VALORES DE ACLERACION OBTENIDOS EN CADA EJE
  //   // Now we'll calculate the accleration value into actual g's
  //   ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
  //   ay = (float)accelCount[1]*aRes - accelBias[1];   
  //   az = (float)accelCount[2]*aRes - accelBias[2]; 
  //   */

  //   if(SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {
  //   Serial.println("Pass Selftest!");  
      
  //   mpu.calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
   //}

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
   //Filtro Pasa Alto
  mpu.setHighPassFilter(MPU6050_HIGHPASS_DISABLE);
  //Introduce un retardo segun documentacion
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ); //GYRO OUTPUT RATE is 1KHz when DLPF enabled
  mpu.setSampleRateDivisor(0); //Sample Rate = 1KHz/(1 + SMPLRT_DIV)

  //VER COMO HACER SELFTEST Y CALIBRACION DEL SENSOR MPU6050
  //REVISAR LIBRERIA DEL SENOR DE GITHUB QUE HACE SELFTEST Y CALIBRACION Y AJUSTAR EL CODIGO AQUI

}

void setup() {


  //se configura puerto serial
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  //Creacion de las colas
  data_temphumQueue = xQueueCreate(1, sizeof(BMEData));
  aclQueue = xQueueCreate(3, sizeof(ACLData));
  bufferQueue = xQueueCreate(3, sizeof(ACLData));

  // Initialize the BME280 sensor
  while(!bme.begin(0x76)) {
    Serial.println("Could not find BME280 sensor!");
    while (1){
      delay(10);
    }
  }

  //bme.setSampling();

  setup_acl_MPU6050();

  Wire.setClock(400000);

  //Creacion de tareas
    //Tareas a ejecutarse en el Nucleo 1
    xTaskCreatePinnedToCore(readBMETask, "readBMETask", 1024*2, NULL, 1, &xHandle_readBMETask, 0);
    xTaskCreatePinnedToCore(receive_temphum, "receiveDataTask", 1024*2, NULL, 1, &xHandle_receive_temphum, 0);
    xTaskCreatePinnedToCore(blink, "readBMETask", 1024, NULL, 1, &xHandle_blink, 0);
    vTaskSuspend(xHandle_blink);
  
   
    //Tareas a ejecutarse en el Nucleo 0
    xTaskCreatePinnedToCore(leerDatosACL, "leerDatosACL", 1024*2, NULL, 2, &xHandle_leerDatosACL, 0);
    xTaskCreatePinnedToCore(crearBuffer, "crearBuffer", 1024*2, NULL, 2, &xHandle_crearBuffer, 0);
}

void loop() {
  // Do nothing
}


// void guardarSD(void *pvParameters){
//   while(true){

//   float bufferXX[NUM_DATOS - 1] = { };  

//   //BufferACL datos_listos;
//   // Initialize SdFat or print a detailed error message and halt
//   // Use half speed like the native library.
//   // change to SPI_FULL_SPEED for more performance.
//   if (!sd.begin(CS, SPI_HALF_SPEED)) sd.initErrorHalt();

//   // open the file for write at end like the Native SD library
//   if (!myFile.open("pruebaTEG1.txt", O_RDWR | O_CREAT | O_AT_END)) {
//     sd.errorHalt("opening test.txt for write failed");
//   }

//   // if the file opened okay, write to it:

//   //Headers
//   myFile.println("ACLX");

//   Serial.print("Se empieza a escribir en memoria...");
//   if(xQueueReceive(bufferQueue, &bufferXX, portMAX_DELAY)){
//      for(int w = 0; w < NUM_DATOS; w++){
//       myFile.print(bufferXX[w]);
//       myFile.print(",");
//      }
//   }
//   //SUSPENDER TODO MIENTRAS SE GUARDA EN SD? O SE SIGUEN ADQUIRIENDO DATOS?

//   // close the file:
//   myFile.close();
//   Serial.println("Se termino de guardar en SD.");
//   }
// }