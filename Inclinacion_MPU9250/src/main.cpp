#include "MPU9250.h"
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

TaskHandle_t xHandle_Task1;
TaskHandle_t Task2;

QueueHandle_t xQueue;
QueueHandle_t xQueue2;

float GyroX, GyroY, GyroZ;
float RateRoll, RatePitch;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch; //Giro en X y en Y
float KalmanAngleRoll = 0, KalmanUncAngleRoll = 2*2; //2 grados prediccion inicial de la incertidumbre al cuadrado
float KalmanAnglePitch = 0, KalmanUncAnglePitch = 2*2;
float Kalman1DOutput[] = {0,0};


MPU9250 mpu;

//Estructura para almacenar los valores de yaw, pitch y roll
struct Data{
  float yaw;
  float pitch;
  float roll;
};


//Variables para el Delay Until
TickType_t xLastWakeTime;

//Funcion para medir cada cuanto se ejecuta Task1
void Task2code( void * pvParameters ){
  for(;;){
    vTaskResume(xHandle_Task1);
    vTaskDelayUntil(&xLastWakeTime, 1000 / portTICK_PERIOD_MS);
    vTaskSuspend(xHandle_Task1);
  }
}


//Tarea de FreeRTOS para guardar los valores de yaw, pitch y roll


void Task1code( void * pvParameters ){
  for(;;){
    if(mpu.update()){
      //Llenando la estructura 
      Data data;
      data.yaw = mpu.getYaw();
      data.pitch = mpu.getPitch();
      data.roll = mpu.getRoll();

      //Enviando los valores a la cola
      //xQueueSend(xQueue, &data, portMAX_DELAY);

      //Imprimiendo los valores de Yaw, Pitch y Roll
        Serial.println("Roll Pitch Yaw");
        Serial.print(mpu.getRoll(), 2);
        Serial.print(" ");
        Serial.print(mpu.getPitch(), 2);
        Serial.print(" ");
        Serial.println(mpu.getYaw(), 2);

        vTaskDelayUntil(&xLastWakeTime, 80 / portTICK_PERIOD_MS);
    }
  
  //vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

//Creacion de buffer para guardar datos
float buffer[1000];

//Tarea de FreeRTOS para recibir los valores en una cola y guardarlos en un buffer
// void Task2code( void * pvParameters ){
//   for(;;){
//     //Recibiendo valores de la cola
//     Data data;
//     xQueueReceive(xQueue, &data, portMAX_DELAY);
  
//     //Guardando valores de yaw pitch y roll en un buffer
//     buffer[0] = data.yaw;
//     buffer[1] = data.pitch;
//     buffer[2] = data.roll;

//   }
// }

void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}

void print_roll_pitch_yaw()
{
  //Print Roll Pitch and Yaw in columns
  Serial.println("Roll Pitch Yaw");
  Serial.print(mpu.getRoll(), 2);
  Serial.print(" ");
  Serial.print(mpu.getPitch(), 2);
  Serial.print(" ");
  Serial.println(mpu.getYaw(), 2);

  // Serial.print(">Yaw:");
  // Serial.println(mpu.getYaw(), 2);
  // Serial.print(">Pitch:");
  // Serial.println(mpu.getPitch(), 2);
  // Serial.print(">RollMadgwick:");
  // Serial.println(mpu.getRoll(), 2);
}

void print_acceleration()
{
  Serial.print(">AclX:");
  Serial.println(mpu.getAccX());
  Serial.print(">AclY:");
  Serial.println(mpu.getAccY());
  Serial.print(">AclZ:");
  Serial.println(mpu.getAccZ());
}

void print_rollrate()
{
  Serial.print(">RollRateX:");
  Serial.println(mpu.getGyroX());
  Serial.print(">RollRateY:");
  Serial.println(mpu.getGyroY());
  Serial.print(">RollRateZ:");
  Serial.println(mpu.getGyroZ());
}

void printAngles(){
  //Getting Euler angles  
  Serial.print(">EulerX [deg]:");
  Serial.println(mpu.getEulerX());
  Serial.print(">EulerY [deg]:");
  Serial.println(mpu.getEulerY());
  Serial.print(">EulerZ [deg]:");
  Serial.println(mpu.getEulerZ());
}


void medirACL_Gyro(){
  //Obtengo mediciones de aceleracion
  AccX = mpu.getAccX();
  AccY = mpu.getAccY();
  AccZ = mpu.getAccZ();
  
  //Obtengo mediciones de giroscopio
  GyroX = mpu.getGyroX(); //
  RateRoll = GyroX;
  GyroY = mpu.getGyroY();
  RatePitch = GyroY;
  GyroZ = mpu.getGyroZ();
}

void AngulosMetodo1(){
  //Calculo angulos de pitch and roll
  AngleRoll = atan(AccY/sqrt(AccX*AccX + AccZ*AccZ))*(1/(3.142/180));
  AnglePitch = -atan(AccX/sqrt(AccY*AccY + AccZ*AccZ))*(1/(3.142/180));
}

void kalman_1d(float KalmanState, float KalmanUnc, float  KalmanInput, float KalmanMeasurement)
{

  /*
KalmanState = Angulo calculado con el filtro de Kalman
KalmanUnc = Incertidumbre de la prediccion
KalmanInput = Rotation Rate del giroscopio
KalmanMeasurement = Angulo calculado con trigonometria y acelerometro
*/

  //Prediccion de estado actual usando estado inicial y entrada
  KalmanState = 1*KalmanState + 0.004*KalmanInput;

  //Calcular incertidumbre de la prediccion
  KalmanUnc = 1*KalmanUnc*1 + 0.004*0.004*4*4;

  //Calcular a ganancia de Kalman
  float KalmanGain = KalmanUnc *1/(1*KalmanUnc + 0.004*0.004*3*3);

  //Actualizar la prediccion con medicion del estado
  KalmanState = KalmanState + KalmanGain*(KalmanMeasurement - 1*KalmanState);

  //Actualizar la incertidumbre de la prediccion
  KalmanUnc = (1 - KalmanGain*1)*KalmanUnc;

  //Salida del filtro de Kalman
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUnc;
}

void AngulosMetodo2(){
  //Calculo angulos de pitch and roll
  AngleRoll = atan(AccY/sqrt(AccX*AccX + AccZ*AccZ))*(1/(3.142/180));
  AnglePitch = -atan(AccX/sqrt(AccY*AccY + AccZ*AccZ))*(1/(3.142/180));

  kalman_1d(KalmanAngleRoll, KalmanUncAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncAngleRoll = Kalman1DOutput[1];

  kalman_1d(KalmanAnglePitch, KalmanUncAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncAnglePitch = Kalman1DOutput[1];
}


long int time_elapsed = 0;
long int tiempo1 = 0;
long int tiempo2 = 0;

uint32_t LoopTimer = 0;
void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    Wire.setClock(400000);

    //Creacion de queue
    xQueue = xQueueCreate(10, sizeof(Data));

    if (!mpu.setup(0x69)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    //calibrate anytime you want to
    Serial.println("Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);
    delay(5000);
    mpu.calibrateAccelGyro();

    // Serial.println("Mag calibration will start in 5sec.");
    // Serial.println("Please Wave device in a figure eight until done.");
    // delay(5000);
    // mpu.calibrateMag();

    print_calibration();
    mpu.verbose(false);

    //Creando tareas de FreeRTOS en el nucleo 0
    xTaskCreatePinnedToCore(
      Task1code,   /* Funcion que implementa la tarea. */
      "Task1",     /* Nombre de la tarea. */
      10000,       /* Tamaño de la pila de la tarea */
      NULL,        /* Parametros de la tarea */
      1,           /* Prioridad de la tarea */
      &xHandle_Task1,      /* Manejador de la tarea */
      0);          /* Nucleo donde se ejecutara la tarea */

    // xTaskCreatePinnedToCore(
    //   Task2code,   /* Funcion que implementa la tarea. */
    //   "Task2",     /* Nombre de la tarea. */
    //   10000,       /* Tamaño de la pila de la tarea */
    //   NULL,        /* Parametros de la tarea */
    //   1,           /* Prioridad de la tarea */
    //   &Task2,      /* Manejador de la tarea */
    //   0);          /* Nucleo donde se ejecutara la tarea */
}

void loop() {

    //Calculando el tiempo que tarda en ejecutarse la funcion mpu.update_accel_gyro

    // if(mpu.update()){
    //   mpu.update_accel_gyro(); //Actualiza valores de aceleracion y giroscopio
    //   medirACL_Gyro(); //Obtengo mediciones de aceleracion y giroscopio
    //   //AngulosMetodo1(); //Calcular los angulos de giro en X y Y usando la aceleracion
    //   AngulosMetodo2(); //Calculo los angulos usando Kalman
    //   // Serial.print(">Roll [deg]:");
    //   // Serial.println(AngleRoll);
    //   // // Serial.print(">Pitch [deg]:");
    //   // // Serial.println(AnglePitch);

    //   // //Ahora los resultados por Kalman
    //   // Serial.print(">Roll Kalman [deg]:");
    //   // Serial.println(KalmanAngleRoll);
    //   // // Serial.print(">Pitch Kalman [deg]:");
    //   // // Serial.println(KalmanAnglePitch);

    //   // while(micros() - LoopTimer < 4000){
    //   //     LoopTimer = micros();
    //   // }


    //   static uint32_t prev_ms = millis();
    //     if (millis() > prev_ms + 20) {

    //       //Madgwick
    //         print_roll_pitch_yaw();

    //         Serial.print(">Roll [deg]:");
    //         Serial.println(AngleRoll);
    //         // Serial.print(">Pitch [deg]:");
    //         // Serial.println(AnglePitch);

    //         //Ahora los resultados por Kalman
    //         Serial.print(">Roll Kalman [deg]:");
    //         Serial.println(KalmanAngleRoll);
    //         // Serial.print(">Pitch Kalman [deg]:");
    //         // Serial.println(KalmanAnglePitch);

    //         prev_ms = millis();
    //       }
    // }
    
    // tiempo1 = micros();
    // if (mpu.update()) {
    //   tiempo2 = micros();
    //   time_elapsed = tiempo2 - tiempo1;
    //   Serial.print("Time elapsed [us]:");
    //   Serial.println(time_elapsed);
    //     static uint32_t prev_ms = millis();
    //     if (millis() > prev_ms + 25) {
    //         print_roll_pitch_yaw();
    //         //print_acceleration();
    //         //print_rollrate();
    //         //printAngles();
    //         prev_ms = millis();
    //     }
    // }
}
