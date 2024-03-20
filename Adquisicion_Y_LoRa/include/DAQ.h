#include <tasks_queues.h>

#define LED_IDLE 17
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

#define NUM_DATOS 1024
#define MEAN_INTERVAL 5
#define NUM_DATOS_TEMP 200
#define NUM_DATOS_INC 800

//Valores limite 15 aceleracion en m/s^2
#define LIM_ACLX 2.0
#define LIM_ACLY 2.0
#define LIM_ACLZ 2.0

//Estructuras de datos 
struct BufferTempHumedad{
  float buffertemp[NUM_DATOS_TEMP/5 - 1] = { };
  float bufferhum[NUM_DATOS_TEMP/5 - 1] = { };
};

//Estructura que contiene 3 arreglos de 5000 flotantes
struct BufferACL{
  long int buffer_timestamp[NUM_DATOS] = {};
  float bufferX[NUM_DATOS] = { };
  float bufferY[NUM_DATOS] = { };
  float bufferZ[NUM_DATOS] = { };
};

struct BufferInclinacion{
  float bufferRoll[NUM_DATOS_INC/5 - 1] = { };
  float bufferPitch[NUM_DATOS_INC/5 - 1] = { };
  float bufferYaw[NUM_DATOS_INC/5 - 1] = { };
};

struct trama_LoRa{
  float trama_final[3*(NUM_DATOS - 1) + 2 + 2]; //Todos los datos de ACL, 2 de INC y 2 de TEMP
};

//Estructura de 2 flotantes para temp&hum
struct BMEData{
  float temperature;
  float humidity;
};

//Estructura de 3 flotantes para cada eje del accl
struct ACLData{
  float AclX;
  float AclY;
  float AclZ;
};

//Estructura de 3 flotantes para cada eje del accl
struct IncData{
  float IncRoll;
  float IncPitch;
  float IncYaw;
} ;

//Prototipos de funciones
int evaluar_limites_acl(float aclX, float aclY, float aclZ);
void leerDatosACL(void *pvParameters);
void crearBuffer(void *pvParameters);
void readBMETask(void *pvParameters);
void receive_temphum(void *pvParameters);
void readMPU9250(void *pvParameters);
void recInclinacion(void *pvParameters);
void blink(void *pvParameters);
void setup_acl_MPU6050();
void setup_mpu9250();
