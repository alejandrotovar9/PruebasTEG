#define NUM_DATOS 4096
#define MEAN_INTERVAL 5
#define NUM_DATOS_TEMP 200
#define NUM_DATOS_INC 800

//Valores limite 15 aceleracion en m/s^2
#define LIM_ACLX 15.0
#define LIM_ACLY 15.0
#define LIM_ACLZ 15.0

//Estructuras de datos 
struct BufferTempHumedad{
  float buffertemp[NUM_DATOS_TEMP/5 - 1] = { };
  float bufferhum[NUM_DATOS_TEMP/5 - 1] = { };
};

//Estructura que contiene 3 arreglos de 5000 flotantes
struct BufferACL{
  long int buffer_timestamp[NUM_DATOS - 1] = {};
  float bufferX[NUM_DATOS - 1] = { };
  float bufferY[NUM_DATOS - 1] = { };
  float bufferZ[NUM_DATOS - 1] = { };
};

struct BufferInclinacion{
  float bufferRoll[NUM_DATOS_INC/5 - 1] = { };
  float bufferPitch[NUM_DATOS_INC/5 - 1] = { };
  float bufferYaw[NUM_DATOS_INC/5 - 1] = { };
};