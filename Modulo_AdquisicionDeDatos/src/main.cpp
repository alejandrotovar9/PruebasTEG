#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <MPU9250.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <Wire.h>
#include <Arduino.h>

#define LED_IDLE 23
#define LED_EST1 4
#define LED_CAL 15
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
#define T_SAMPLING_INC 100     //frecuencia = Hz

#define NUM_DATOS 4096
#define MEAN_INTERVAL 5
#define NUM_DATOS_TEMP 200
#define NUM_DATOS_INC 800

//Valores limite 15 aceleracion en m/s^2
#define LIM_ACLX 2.0
#define LIM_ACLY 2.0
#define LIM_ACLZ 2.0

//Sensor objects
Adafruit_BME280 bme;
Adafruit_MPU6050 mpu;
MPU9250 mpu9250;

// Task handle
TaskHandle_t xHandle_readBMETask = NULL;
TaskHandle_t xHandle_receive_temphum = NULL;

TaskHandle_t xHandle_leerDatosACL = NULL;
TaskHandle_t xHandle_crearBuffer = NULL;
TaskHandle_t xHandle_recibirDatosACL = NULL;

TaskHandle_t xHandle_readMPU9250 = NULL;
TaskHandle_t xHandle_recInclinacion= NULL;

TaskHandle_t xHandle_blink = NULL;


//Handle de la Cola
QueueHandle_t xQueue;
QueueHandle_t data_temphumQueue;
QueueHandle_t aclQueue;
QueueHandle_t bufferQueue;
QueueHandle_t incQueue;

//Variable de eventos MPU6050
sensors_event_t a, g, tem;

//Estructuras de datos

struct BufferTempHumedad{
  float buffertemp[NUM_DATOS_TEMP/5 - 1] = { };
  float bufferhum[NUM_DATOS_TEMP/5 - 1] = { };
};
BufferTempHumedad estruc_buffer_datos;

//Estructura que contiene 3 arreglos de 5000 flotantes
struct BufferACL{
  long int buffer_timestamp[NUM_DATOS - 1] = {};
  float bufferX[NUM_DATOS - 1] = { };
  float bufferY[NUM_DATOS - 1] = { };
  float bufferZ[NUM_DATOS - 1] = { };
};
BufferACL struct_buffer_acl;

struct BufferInclinacion{
  float bufferRoll[NUM_DATOS_INC/5 - 1] = { };
  float bufferPitch[NUM_DATOS_INC/5 - 1] = { };
  float bufferYaw[NUM_DATOS_INC/5 - 1] = { };
};
BufferInclinacion estruc_buffer_inclinacion;

struct trama_LoRa{
  float trama_final[3*(NUM_DATOS - 1) + 2 + 2]; //Todos los datos de ACL, 2 de INC y 2 de TEMP
};
static trama_LoRa trama_final;

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

//Estructura de 3 flotantes para cada eje del accl
struct IncData{
  float IncRoll;
  float IncPitch;
  float IncYaw;
};


//Variables globales
int k = 0; //Contador para la cantidad de datos a guardar en el array
long int time_elapsed = 0;
long int tiempo1 = 0;
long int tiempo2 = 0;

float gyroBias[3], accelBias[3]; // Bias corrections for gyro and accelerometer

int w = 0; //Contador para la cantidad de datos a guardar en el buffer de temperatura y humedad

//Variables globales para medicion de temperatura y humedad
float prom_temp = 0;
float prom_hum = 0;
int cont = 1; //Variable interna de iteraciones
int cont2 = 0; //Posicion del buffer en la que se va a guardar

//Variables globales para medicion de inclinacion
float prom_roll = 0, prom_pitch = 0, prom_yaw = 0;
int cont_inc = 0, cont2_inc = 1; //Variables internas de iteracion y posicion del buffer

//Variables para el Delay Until
TickType_t xLastWakeTime;

//Banderas para control de tareas
int flag_limite = 0;
int flag_temp_hum = 0;
int flag_inc = 0;

//Calibracion
float acl_offset[3];

//Devuelve una structura de tipo Trama Lora
//Las input llevan const para evitar que sean modificadas las estrucutras por buenas practicas
//s1 es una referencia a un objeto de tipo BufferACL
trama_LoRa combineArrays(const BufferACL& s1, int len1, float* s2 ,int len2, float* s3, int len3) {
    trama_LoRa trama;

    memcpy(trama.trama_final, s1.bufferX, sizeof(s1.bufferX));
    memcpy(trama.trama_final + len1, s1.bufferY, sizeof(s1.bufferY));
    memcpy(trama.trama_final + len1 + len1, s1.bufferZ, sizeof(s1.bufferZ));
    memcpy(trama.trama_final + len1 + len1 + len2, s2, sizeof(s2));
    memcpy(trama.trama_final + len1 + len1 + len2 + len3, s3, sizeof(s3));

    return trama;
}

int evaluar_limites_acl(float aclX, float aclY, float aclZ){  
  if(aclX >= LIM_ACLX){
    printf("El acelerometro en el eje X supero los %f m/s \n", LIM_ACLX);
    return 1;
  }
  if(aclY >= LIM_ACLY){
    printf("El acelerometro en el eje Y supero los %f m/s \n", LIM_ACLY);
    return 2;
  }
  if(aclZ > LIM_ACLZ){
    printf("El acelerometro en el eje Z supero los %f m/s \n", LIM_ACLZ);
    return 3;
  }
  else{
    return 0;
  }
}

void leerDatosACL(void *pvParameters){
  //Leer los valores del Acelerometro de la IMU
  while(true){

      mpu.getEvent(&a, &g, &tem);

      ACLData aclData; //Estructura a ser llenada con 3 ejes

      //Se llena estructura con datos nuevos en m/s^2 ya que estan multiplicados por la ctte SENSORS_GRAVITY_EARTH, si se quieren en g se divide por esta constante

      /*/ SENSORS_GRAVITY_EARTH
        / SENSORS_GRAVITY_EARTH
        / SENSORS_GRAVITY_EARTH*/
      //Eliminando el offset solo si es mayor a +- 0.02

      if(acl_offset[0] < 0){
        acl_offset[0] = -acl_offset[0];
      }
      else if(acl_offset[1] < 0){
        acl_offset[1] = -acl_offset[1];
      }
      else if(acl_offset[2] < 0){
        acl_offset[2] = -acl_offset[2];
      }

      
      aclData.AclX = a.acceleration.x - acl_offset[0];
      aclData.AclY = a.acceleration.y - acl_offset[1]; 
      aclData.AclZ = a.acceleration.z - acl_offset[2];

      //La data esta llenando los registros a 1KHz por la configuracion del sensor, sin embargo, se esta muestreando mas lento por la naturaleza de los sistemas en estudio

      //COMO SUSTITUIR ESTE DELAY??? LEER SOLO CUANDO LA DATA ESTE LISTA
      if(F_SAMPLING != 0){
        vTaskDelay(F_SAMPLING/portTICK_PERIOD_MS); //Reducir tiempo de muestreo
        //xTaskDelayUntil
      }
      
      //Evaluo los valores actuales de aceleracion
      if(flag_limite == 0){
        flag_limite = evaluar_limites_acl(aclData.AclX, aclData.AclY, aclData.AclZ);
        if (flag_limite != 0){
          //Se ejecuta una sola vez
          Serial.println("Se supero el limite de aceleracion, tomar accion!!!");
          //Activando banderas para registro de temperatura y humedad
          flag_inc = 1;
          flag_temp_hum = 1;
          digitalWrite(LED_EST1, HIGH);
          vTaskSuspend(xHandle_blink);
        }
      }

      //Envia los datos a la cola solo si se supero el limite
      if(flag_limite != 0){
        //Se supero un limite tomar accion
        //Serial.println("Se supero un limite de aceleracion, tomar accion!!!");
        //Comienza el registro de datos
        if(xQueueSend(aclQueue, &aclData, portMAX_DELAY)){
        //Serial.println("Se envio la cola con datos...");
        vTaskResume(xHandle_crearBuffer); //Se llenan los buffers para enviar los datos
        }
        else{
          Serial.println("No se envio la cola...");
        }
      }
      else{
        //Datos dentro de valores normales
        continue;
      }

      //Envia los datos a la cola aclQueue
      // if(xQueueSend(aclQueue, &aclData, portMAX_DELAY)){
      //   //Serial.println("Se envio la cola con datos...");
      //   vTaskResume(xHandle_crearBuffer); //Se llenan los buffers para enviar los datos
      // }
      // else{
      //   Serial.println("No se envio la cola...");
      // }

      //vTaskResume(xHandle_crearBuffer); //Reinicia la tarea para crear buffer
  }
}

void mostrar_resultados(void){
  for(int w = 2037; w < 2047; w++){
          printf("Valores de tiempo: %u, X: %f Y: %f Z: %f \n", struct_buffer_acl.buffer_timestamp[w], struct_buffer_acl.bufferX[w], struct_buffer_acl.bufferY[w], struct_buffer_acl.bufferZ[w]);
        }

        time_elapsed = (struct_buffer_acl.buffer_timestamp[NUM_DATOS-1] - struct_buffer_acl.buffer_timestamp[0]);
        printf("Tiempo transcurrido: %d milisegundos \n", time_elapsed);

        Serial.println();
        Serial.println("Buffer de temperatura y humedad");
        for(int m = 0; m < 15; m++) {
          Serial.print("Temperatura: ");
          Serial.print(estruc_buffer_datos.buffertemp[m]);
          Serial.print(" ");  
          Serial.print("Humedad: ");
          Serial.print(estruc_buffer_datos.bufferhum[m]);
          Serial.println(" "); 
        }
        //Imprimiendo de igual forma los valores de inclinacion
        Serial.println("Buffer de inclinacion");
        for(int j = 0; j < 15; j++) {
          Serial.print("Pitch: ");
          Serial.print(estruc_buffer_inclinacion.bufferPitch[j]);
          Serial.print(" ");  
          Serial.print("Roll: ");
          Serial.print(estruc_buffer_inclinacion.bufferRoll[j]);
          Serial.print("Yaw: ");
          Serial.print(estruc_buffer_inclinacion.bufferYaw[j]);
          Serial.println(" "); 
        }
}

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
        k++;
      }
      else{
        Serial.println("Se termino de llenar la estructura con exito!!!");

        // Serial.println("Suspendiendo tarea de recepcion de datos de temperatura...");
        // vTaskSuspend(xHandle_readBMETask); //Suspendo la tarea de recepcion de datos de temperatura
        // Serial.println("Suspendiendo la tarea de recepcion de datos de inclinacion...");
        // vTaskSuspend(xHandle_readMPU9250);
        Serial.println("Reiniciando banderas de limite...");

        flag_limite = 0;
        flag_inc = 0;
        flag_temp_hum = 0;

        //Apago led indicativo de toma de datos
        digitalWrite(LED_EST1, LOW);

         mostrar_resultados();
       

        //CAUSA REINICIO
        // float aux_array_inc[2];
        // float aux_array_temp[2];

        // //Fill the auxiliary arrays with values from position 2 onwards
        // for(int y = 0; y < 2; y++) {
        //   aux_array_inc[y] = estruc_buffer_inclinacion.bufferPitch[y+2];
        //   aux_array_temp[y] = estruc_buffer_datos.buffertemp[y+2];
        // }

        // trama_final = combineArrays(struct_buffer_acl, NUM_DATOS-1, aux_array_inc, 2, aux_array_temp, 2);

        //Envio los resultados a la tarea LORA-----------------------------------------

        //Creando trama de datos para enviar por LORA

        //Reiniciando para sobreescribir en buffers "nuevos" en la siguiente accion
        k = 0;
        cont2 = 0;
        cont2_inc = 0;

        vTaskResume(xHandle_blink);
      }
    }
    else{
      Serial.println("No se recibio la cola correctamente...");
    };
 
    //Se suspende esta tarea para esperar la toma de datos
    vTaskSuspend(xHandle_crearBuffer);

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

    //Envia los datos a la cola dataQueue si bandera esta activada
    if(flag_temp_hum){
        if(xQueueSend(data_temphumQueue, &data_readtemp, portMAX_DELAY)){
          //Serial.println("Se envio correctamente la cola de temperatura");
          vTaskDelay(10 / portTICK_PERIOD_MS); //Este delay permite que las lecturas del sensor sean las correctas
          vTaskResume(xHandle_receive_temphum); //Reactivo tarea de creacion de buffers y promedios
        }
        else{
          Serial.println("No se envio correctamente la cola de temperatura");
        }
    }
    
    //Delay dependiendo del tiempo de muestreo
    vTaskDelay(T_SAMPLING_TEMPHUM / portTICK_PERIOD_MS); //5Hz
  }
}

float valor_temp_anterior = 0;
float valor_hum_anterior = 0;

// //Tarea de recepcion de datos
void receive_temphum(void *parameter) {
  while (true) {
    //Recibe la data de la cola y la guarda en una nueva estructura
    BMEData data_temphum;
    
    float valor_temp_actual = 0;
    float valor_hum_actual = 0;

    if(xQueueReceive(data_temphumQueue, &data_temphum, portMAX_DELAY)){

    //CALCULO DEL PROMEDIO

        //Implementacion del filtro de primer orden
        if(cont == 1){
          valor_temp_anterior = data_temphum.temperature;
          valor_hum_anterior = data_temphum.humidity;

          //Guardo las mediciones actuales en la primera iteracion
          valor_temp_actual = data_temphum.temperature;
          valor_hum_actual = data_temphum.humidity;
        }
        else{
          //Actualizo valor actual con pesos de primer orden dandole mas peso al valor anterior
          valor_temp_actual = valor_temp_anterior*0.9 + data_temphum.temperature*0.1;
          valor_hum_actual = valor_hum_anterior*0.9 + data_temphum.humidity*0.1;
          //Actualizo valores anteriores
          valor_temp_anterior = valor_temp_actual;
          valor_hum_anterior = valor_hum_actual;
        }

        //Guardo las mediciones actuales
        // valor_temp_actual = data_temphum.temperature;
        // valor_hum_actual = data_temphum.humidity;

        //Las agrego al promedio
        prom_temp = valor_temp_actual + prom_temp;
        prom_hum = valor_hum_actual + prom_hum;

        if(cont >= MEAN_INTERVAL){
          // printf("El  actual de temperatura es: %f \n", valor_temp_actual);
          // printf("El  actual de humedad es: %f \n", valor_hum_actual);
          //printf("El contador actual para calcular el promedio es: %i \n", cont);
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

//Tarea de recepcion de datos de MPU9250
void readMPU9250(void *pvParameters){
  while (true) {
      //Crea una estructura de tipo IncData
      IncData data_inc;
       //Lee los valores del sensor y los guarda en la estructura

       if(mpu9250.update()){
          data_inc.IncRoll = mpu9250.getRoll();
          data_inc.IncPitch = mpu9250.getPitch();
          data_inc.IncYaw = mpu9250.getYaw();

          //Evaluacion de limites de valores de inclinacion

          //Envia los datos a la cola incQueue solo si la bandera esta activada
          if(flag_inc){
            if(xQueueSend(incQueue, &data_inc, portMAX_DELAY)){
            //Serial.println("Se envio correctamente la cola de inclinacion");
            vTaskResume(xHandle_recInclinacion); //Reactivo tarea de creacion de buffers y promedios
            }
            else{
              Serial.println("No se envio correctamente la cola de inclinacion");
            }
          }
          vTaskDelayUntil(&xLastWakeTime, T_SAMPLING_INC / portTICK_PERIOD_MS)
       }

    //vTaskDelay(T_SAMPLING_INC / portTICK_PERIOD_MS); //5Hz
    }
}

//Tarea de recepcion de datos de Inclinacion y calculo del promedio
void recInclinacion(void *pvParameters){
  while (true) {
    //Recibe la data de la cola y la guarda en una nueva estructura
    IncData data_inclinacion;
    
    float valor_yaw_actual = 0;
    float valor_pitch_actual = 0;
    float valor_roll_actual = 0;

    //Recibo la estructura de la cola
    if(xQueueReceive(incQueue, &data_inclinacion, portMAX_DELAY)){
    //Serial.println("Se recibio la cola!");

    //CALCULO DEL PROMEDIO
        //Guardo las mediciones actuales
        valor_roll_actual = data_inclinacion.IncRoll;
        valor_pitch_actual = data_inclinacion.IncPitch;
        valor_yaw_actual = data_inclinacion.IncYaw;

        //Las agrego al promedio
        prom_roll = valor_roll_actual + prom_roll;
        prom_pitch = valor_pitch_actual + prom_pitch;
        prom_yaw = valor_yaw_actual + prom_yaw;
          // printf("El  actual de roll es: %f \n", valor_roll_actual);
          // printf("El  actual de pitch es: %f \n", valor_pitch_actual);
          // printf("El  actual de yaw es: %f \n", valor_yaw_actual);
          // Serial.println();

        if(cont_inc >= MEAN_INTERVAL * 2){
         //vTaskDelay(1 / portTICK_PERIOD_MS);
          // printf("El  actual de roll es: %f \n", valor_roll_actual);
          // printf("El  actual de pitch es: %f \n", valor_pitch_actual);
          // printf("El  actual de yaw es: %f \n", valor_yaw_actual);
          // Serial.println();

          estruc_buffer_inclinacion.bufferRoll[cont2_inc] = prom_roll / cont_inc;
          estruc_buffer_inclinacion.bufferPitch[cont2_inc] = prom_pitch / cont_inc;
          estruc_buffer_inclinacion.bufferYaw[cont2_inc] = prom_yaw / cont_inc;

          cont_inc = 0; //reinicio el contador de valores obtenidos para sacar promedio
          cont2_inc++; //Sumo 1  a este contador para guardar en siguiente posicion
          //Reinicio promedios de roll, pitch y yaw
          prom_roll = 0;
          prom_pitch = 0;
          prom_yaw = 0;
        }
        cont_inc++;
    }
    else{
      Serial.println("No se recibio la cola correctamente...");
    };
    
    vTaskSuspend(NULL); //Suspendo esta tarea hasta que se vuelva a activar desde otra
  }
}


void blink(void * pvParameters ) {
    while (1) {
        digitalWrite(LED_IDLE, HIGH);
        //Serial.println("Led encendido");
        delay(1000);
        digitalWrite(LED_IDLE, LOW);
        //Serial.println("Led apagado");
        delay(1000);
    }
}


/*--------------------------------SETUP FUNCTIONS--------------------------------*/


void MPU6050Offsets() {
  const int numSamples = 1000; // Number of samples to collect for calibration

  float sumX = 0.0, sumY = 0.0, sumZ = 0.0;
  sensors_event_t a_cal, g_cal, tem_cal;


  // Initialize the MPU6050
  mpu.begin();

  // Calibrate the accelerometer offsets
  for (int i = 0; i < numSamples; i++) {
    mpu.getEvent(&a_cal, &g_cal, &tem_cal);
    sumX += a_cal.acceleration.x;
    sumY += a_cal.acceleration.y;
    sumZ += a_cal.acceleration.z;
    //delay(10); // Adjust the delay based on your requirements
  }

  // Calculate the average offsets
  float avgOffsetX = sumX / numSamples;
  float avgOffsetY = sumY / numSamples;
  float avgOffsetZ = sumZ / numSamples;

  //Save in acl_offset
  acl_offset[0] = avgOffsetX;
  acl_offset[1] = avgOffsetY;
  acl_offset[2] = avgOffsetZ;

  //Print the average offsets
  Serial.print("Average X offset: "); Serial.println(avgOffsetX);
  Serial.print("Average Y offset: "); Serial.println(avgOffsetY);
  Serial.print("Average Z offset: "); Serial.println(avgOffsetZ);

}


void setup_acl_MPU6050(void){

  int16_t accelCount[3];           // Stores the 16-bit signed accelerometer sensor output
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
  Serial.println("6-DOF - 16-bit motion sensor - 60 ug LSB");
  float fmuestreo = 1/(F_SAMPLING*0.001);
  printf("La frecuencia de muestreo escogida es: %.2f Hz \n", fmuestreo);

  digitalWrite(LED_CAL, HIGH);

    mpu.MPU6050SelfTest(SelfTest); // Start by performing self test and reporting values
    Serial.print("Self Test Eje X: Trim de aceleracion entre: "); Serial.print(SelfTest[0],1); Serial.println("% del valor nominal");
    Serial.print("Self Test Eje Y: Trim de aceleracion entre: "); Serial.print(SelfTest[1],1); Serial.println("% del valor nominal");
    Serial.print("Self Test Eje Z: Trim de aceleracion entre: "); Serial.print(SelfTest[2],1); Serial.println("% del valor nominal");
    Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");

    if(SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) 
    {
    Serial.println("El sensor paso el self test!"); 
    // mpu.calibrateMPU6050(gyroBias, accelBias); //Los bias calculados de salida estan en g, por lo que habria que multiplicarlos por SENSORS_GRAVITY para tenerlos en m/s^2
    // Serial.println("Giroscopio y acelerometro calibrados, cargando biases en registros de bias");
    // //Imprimiendo en serial valores de bias del acelerometro
    // Serial.print("Bias del acelerometro en X: "); Serial.println(accelBias[0]);
    // Serial.print("Bias del acelerometro en Y: "); Serial.println(accelBias[1]);
    // Serial.print("Bias del acelerometro en Z: "); Serial.println(accelBias[2]);
    // Serial.println();
    }
    else{
      Serial.println("El sensor no paso el self test, revise el sensor");
    }

  MPU6050Offsets();

    //mpu.calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers,
  //load biases in bias registers  
  

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

void setup_mpu9250(){

      if (!mpu9250.setup(0x69)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    Serial.println("MPU9250 conectado!");

    vTaskDelay(100/portTICK_PERIOD_MS);

    Serial.println("Accel Gyro calibration will start in 1sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu9250.verbose(true);
    delay(500);
    mpu9250.calibrateAccelGyro();

    // Serial.println("Mag calibration will start in 1sec.");
    // Serial.println("Please Wave device in a figure eight until done.");
    // delay(500);
    // mpu9250.calibrateMag();

    //mpu9250.verbose(false);

    digitalWrite(LED_CAL, LOW);

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
  incQueue = xQueueCreate(1, sizeof(IncData));

  //Setting ouput pins for leds
  pinMode(LED_EST1, OUTPUT);
  pinMode(LED_IDLE, OUTPUT);
  pinMode(LED_CAL, OUTPUT);

  // Initialize the BME280 sensor

  /*Parametros iniciales del BME280 estan disponibles en el constructor del Adafruit_BME280.h
    void setSampling(sensor_mode mode = MODE_NORMAL,
                   sensor_sampling tempSampling = SAMPLING_X16,
                   sensor_sampling pressSampling = SAMPLING_X16,
                   sensor_sampling humSampling = SAMPLING_X16,
                   sensor_filter filter = FILTER_OFF,
                   standby_duration duration = STANDBY_MS_0_5);
  */

  while(!bme.begin(0x76)) {
    Serial.println("Could not find BME280 sensor!");
    while (1){
      delay(10);
    }
  }

  //Configuracion del acelerometro MPU6050
  setup_acl_MPU6050();
  //Configuracion del acelerometro 9DOF MPU9250
  setup_mpu9250();

  Wire.setClock(400000); //Cambio en la frecuencia del reloj I2C

  //Creacion de tareas
    //Tareas a ejecutarse en el Nucleo 1
    xTaskCreatePinnedToCore(readBMETask, "readBMETask", 1024*2, NULL, 1, &xHandle_readBMETask, 1);
    xTaskCreatePinnedToCore(receive_temphum, "receiveDataTask", 1024*2, NULL, 1, &xHandle_receive_temphum, 1);
    vTaskSuspend(xHandle_receive_temphum);
    xTaskCreatePinnedToCore(blink, "readBMETask", 1024, NULL, 1, &xHandle_blink, 1);
    //vTaskSuspend(xHandle_blink);
  
    //Tareas a ejecutarse en el Nucleo 0
    xTaskCreatePinnedToCore(leerDatosACL, "leerDatosACL", 1024*2, NULL, 2, &xHandle_leerDatosACL, 0);
    xTaskCreatePinnedToCore(crearBuffer, "crearBuffer", 1024*2, NULL, 2, &xHandle_crearBuffer, 0);
    vTaskSuspend(xHandle_crearBuffer);

    //Tareas para obtener la inclinacion
    xTaskCreatePinnedToCore(readMPU9250, "readMPU9250", 1024*2, NULL, 0, &xHandle_readMPU9250, 1);
    xTaskCreatePinnedToCore(recInclinacion, "recInclinacion", 1024*2, NULL, 0, &xHandle_recInclinacion, 1);
    vTaskSuspend(xHandle_recInclinacion);
}

void loop() {
  delay(1000);
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