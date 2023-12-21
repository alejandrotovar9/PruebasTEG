#include <accl.h>

extern TaskHandle_t xHandle_crearBuffer;
extern TaskHandle_t xHandle_recibirDatosACL;
extern TaskHandle_t xHandle_leerDatosACL;
extern TaskHandle_t xHandle_blink;

extern QueueHandle_t aclQueue;
extern QueueHandle_t bufferQueue;

extern Adafruit_MPU6050 mpu;

//Variable global
extern sensors_event_t a, g, tem;

extern uint8_t buffer_timestamp[NUM_DATOS - 1];
extern float bufferX[NUM_DATOS - 1];
extern float bufferY[NUM_DATOS - 1];
extern float bufferZ[NUM_DATOS - 1];

//Variables globales
int k = 0; //Contador para la cantidad de datos a guardar en el array

void accl_setup(void){

    Serial.println("Adafruit MPU6050 test!");

  //Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
   //Filtro Pasa Alto
  mpu.setHighPassFilter(MPU6050_HIGHPASS_DISABLE);
  //Introduce un retardo segun documentacion
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  mpu.setSampleRateDivisor(0);
}


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

      Serial.println("Se lleno estructura");

      //Envia los datos a la cola dataQueue
      if(xQueueSend(aclQueue, &aclData, portMAX_DELAY)){
        Serial.println("Se envio la cola con datos...");
        vTaskResume(xHandle_crearBuffer);
      }
      else{
        Serial.println("No se envio la cola...");
      }

      //vTaskResume(xHandle_crearBuffer); //Reinicia la tarea para crear buffer

      //Para ejecutar una sola vez mas para ver contenidos
      // if(k<=5001){
      //   vTaskResume(xHandle_crearBuffer); //Reinicia la tarea para crear buffer
      // }
  }
}


void crearBuffer(void *pvParameters){
  while(true){

    //Buffer a llenar
    BufferACL buffer;
    ACLData datos_acl;

    //Recibo los datos de la cola y los guardo en la estructura creada
    if(xQueueReceive(aclQueue, &datos_acl, portMAX_DELAY)){
      if(k < NUM_DATOS){
        
        // buffer.buf_tstamp[k] = millis();
        // buffer.bufX[k] = datos_acl.AclX;
        // buffer.bufY[k] = datos_acl.AclX;
        // buffer.bufZ[k] = datos_acl.AclX;
        //SI SE USA ESTRUCTURA NO SE PUEDEN VISUALIZAR VALORES, SE ACTIVA WDT
        //PARECE QUE NO SE ESTA LLENANDO DE DATOS CADA ARRAY DE LA STRUCT

        buffer_timestamp[k] = millis();
        bufferX[k] = datos_acl.AclX;
        bufferY[k] = datos_acl.AclY;
        bufferZ[k] = datos_acl.AclZ;

        // printf(">ACLX: %f \n", bufferX[k]);
        // printf(">ACLY: %f \n", bufferY[k]);
        // printf(">ACLZ: %f \n", bufferX[k]);

        k++; 
        printf("k: %u \n", k);
        //Serial.println(k);
      }
      else{
        Serial.println("Se termino de llenar la estructura con exito!!!");
        printf("Valor final de k: %u \n", k);

        //Inicia el parpadeo del LED en el otro nucleo
        vTaskResume(xHandle_blink);

        for(int w = 0; w < 50; w++){
          printf("t: %u, X: %f, Y: %f, Z: %f \n", buffer_timestamp[w], bufferX[w], bufferY[w], bufferZ[w]);
        }

        xQueueSend(bufferQueue, &buffer, portMAX_DELAY);

        vTaskDelete(xHandle_crearBuffer);
      }
    }
    else{
      Serial.println("No se recibio la cola correctamente...");
    };

    //Se suspende la tarea para esperar la toma de datos
    vTaskSuspend(xHandle_crearBuffer);
    //printf("BUFFER TASK: ESTOY AQUI TODAVIA \n");
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
    Serial.print(">Acceleration Y:");
    Serial.println(aclData2.AclY);
    Serial.print(">Acceleration Z:");
    Serial.println(aclData2.AclZ);
  }
}

//Crear tarea para tomar cierta cantidad de datos (5000 por ej) y luego terminar el programa
//Para evitar el delay al llenar la SD mientras se adquieren los datos se puede usar un buffer temporal
//Luego de guardan los datos del buffer en SD y esto puede ejecutarse en el nucleo 2 del ESP
/*
Ejemplo:

Luego en otra tarea accedo a esta estructura y guardo los datos en SD con el formato CSV deseado.
Ejecutando esta tarea en otro nucleo sin afectar el tiempo de muestreo.
Luego limpio el buffer y lo vuelvo a llenar y as sucesivamente.

Preguntas:
-Cual es el mejor tipo de datos a usar? Array de flotantes
-Esto se guarda en RAM? Es decir, la estructura de datos a actualizarse, debe ser...
-Cuanto tiempo tarda esto en ejecutarse? El llenado de la estructura no debería tomar mucho tiempo, y 
el guardado en SD al estarse ejecutando en el otro nucleo no debería ser problema.
*/

//MPU-6050 da los valores en enteros de 16 bits
//Valores RAW
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
//Ratios de conversion
#define A_R 16384.0 // 32768/2

// void leerDatosACL(void *pvParameters){
//   // /* Get new sensor events with the readings */
//      mpu.getEvent(&a, &g, &tem);
//   // }
//   //Leer los valores del Acelerometro de la IMU
//   while(true){

//       ACLData aclData; //Se define estructura a ser llenada

//       Wire.beginTransmission(0x68);
//       Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
//       Wire.endTransmission(false);
//       Wire.requestFrom(0x68 ,6, true);   //A partir del 0x3B, se piden 6 registros

//       //Se llena estructura con datos nuevos 
//       aclData.AclX = (Wire.read()<<8|Wire.read())/A_R; //Cada valor ocupa 2 registros
//       aclData.AclY = (Wire.read()<<8|Wire.read())/A_R;
//       aclData.AclZ = (Wire.read()<<8|Wire.read())/A_R;

//       //Envia los datos a la cola dataQueue
//       xQueueSend(aclQueue, &aclData, portMAX_DELAY);
//   }
// }

void EnableInt(void *pvParameters){
      Wire.beginTransmission(0x68);  // Initialize the Tx buffer
      Wire.write(0x38);           // Put slave register address in Tx buffer
      Wire.write(0x01);                 // Put data in Tx buffer
      Wire.endTransmission();           // Send the Tx buffer
      //Interrupt activado
}
