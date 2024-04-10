#include <DAQ.h>

//INICIALIZACION DE ESTRUCTURAS AQUI PORQUE ES DONDE LAS NECESITO
BufferTempHumedad estruc_buffer_datos;
BufferACL struct_buffer_acl;
BufferInclinacion estruc_buffer_inclinacion;
//static trama_LoRa trama_final; //Es muy grande

//Variables globales
int k = 0; //Contador para la cantidad de datos a guardar en el array
long int time_elapsed = 0;
long int tiempo1 = 0;
long int tiempo2 = 0;

float gyroBias[3], accelBias[3]; // Bias corrections for gyro and accelerometer
float acl_offset[3]; //Offset del acelerometro

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
int flag_send_packet = 0;
bool flag_acl = false;

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

      aclData.AclX = a.acceleration.x - acl_offset[0];
      aclData.AclY = a.acceleration.y - acl_offset[1]; 
      aclData.AclZ = a.acceleration.z - acl_offset[2];

      //COMO SUSTITUIR ESTE DELAY??? LEER SOLO CUANDO LA DATA ESTE LISTA
      if(F_SAMPLING != 0){
        vTaskDelay(F_SAMPLING/portTICK_PERIOD_MS); //Reducir tiempo de muestreo
        //xTaskDelayUntil
      }
      
      //Evaluo los valores actuales de aceleracion
      if(flag_limite == 0){
        flag_limite = evaluar_limites_acl(aclData.AclX, aclData.AclY, aclData.AclZ);
        if(flag_limite != 0){
          flag_acl == true;

          //Se ejecuta una sola vez al evaluar y verificar que es distinto de 0
          Serial.println("Se supero el limite de aceleracion! Tomando datos...");
          //Activando banderas para registro de temperatura y humedad
          flag_inc = 1;
          flag_temp_hum = 1;

          //Cambio en el LED de toma de datos
          digitalWrite(LED_EST1, HIGH);
          //Suspende tarea de LED IDLE
          vTaskSuspend(xHandle_blink);
        }
      }

      //solo evalua si no se sobrepaso el limite al mismo tiempo
      if(flag_acl == true && flag_limite == 0){
          //Se cambia el valor de flag limite para solo ejecutar esto 1 vez
          flag_limite = 1;

          //Se ejecuta una sola vez al evaluar y verificar que es distinto de 0
          Serial.println("Se recibio peticion de datos por LoRa! Tomando datos...");
          //Activando banderas para registro de temperatura y humedad
          flag_inc = 1;
          flag_temp_hum = 1;

          //Cambio en el LED de toma de datos
          digitalWrite(LED_EST1, HIGH);
          //Suspende tarea de LED IDLE
          vTaskSuspend(xHandle_blink);
        }

      //continue;
      Serial.print(".");

      //Envia los datos a la cola solo si se supero el limite
      if(flag_limite != 0 || flag_acl == true)
      {
        if(xQueueSend(aclQueue, &aclData, portMAX_DELAY)){
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
  for(int w = 0; w < 20; w++){
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

    ACLData datos_acl;

    //Recibo los datos de la cola y los guardo en la estructura creada
    if(xQueueReceive(aclQueue, &datos_acl, portMAX_DELAY)){
      if( k <= NUM_DATOS ){
        if(k == 1) tiempo1 = millis();
        //Lleno el buffer de datos
        struct_buffer_acl.buffer_timestamp[k] = millis();
        struct_buffer_acl.bufferX[k] = datos_acl.AclX;
        struct_buffer_acl.bufferY[k] = datos_acl.AclY;
        struct_buffer_acl.bufferZ[k] = datos_acl.AclZ;
        k++;
      }
      else{
        Serial.println("Se termino de llenar la estructura con exito!!!");
        Serial.print("Valor final de k luego de llenar los datos: ");
        Serial.println(k);

        // Serial.println("Suspendiendo tarea de recepcion de datos de temperatura...");
        // vTaskSuspend(xHandle_readBMETask); //Suspendo la tarea de recepcion de datos de temperatura
        // Serial.println("Suspendiendo la tarea de recepcion de datos de inclinacion...");
        // vTaskSuspend(xHandle_readMPU9250);

        Serial.println("Reiniciando banderas de limite...");

                //Reiniciando para sobreescribir en buffers "nuevos" en la siguiente accion
        k = 0;
        cont2 = 0;
        cont2_inc = 0;

        //Se siguen tomando datos mas no se guardan en los buffers
        flag_limite = 0;
        flag_inc = 0;
        flag_temp_hum = 0;
        flag_acl = false; //Reinicio booleano de recepcion LoRa para toma de decisiones en proxima peticion

        //Apago led indicativo de toma de datos
        digitalWrite(LED_EST1, LOW);

        mostrar_resultados();

        //Envio los resultados a la cola
        if(xQueueSend(tramaLoRaQueue, &struct_buffer_acl, portMAX_DELAY)){
           vTaskSuspend(xHandle_leerDatosACL); //suspendo adquisicion hasta que se envie todo
           vTaskSuspend(xHandle_readBMETask);
           vTaskSuspend(xHandle_readMPU9250);
          Serial.println("Se envio la cola tramaLoRaQueue...");

          //Se envian los datos mediante lora
          vTaskResume(xHandle_send_packet);
          //vTaskResume(xHandle_poll_packet);
        }
        else{
          Serial.println("No se envio la cola...");
        }

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
        
        //Solo guardo a partir de la 2da medicion para evitar la lectura erronea inicial
        //Las agrego al promedio
        prom_temp = valor_temp_actual + prom_temp;
        prom_hum = valor_hum_actual + prom_hum;
        
        if(cont >= MEAN_INTERVAL){
          // printf("El  actual de temperatura es: %f \n", valor_temp_actual);
          // printf("El  actual de humedad es: %f \n", valor_hum_actual);
          //printf("El contador actual para calcular el promedio es: %i \n", cont);
          // if((prom_temp / cont) < 10.0){
          //   estruc_buffer_datos.buffertemp[cont2] = prom_temp / cont;
          //   estruc_buffer_datos.bufferhum[cont2] = prom_hum / cont;
          // }
          // else{
          //   continue;
          // }
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

//Manejo del blink de IDLE
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
void MPU6050Offsets(void) {
  const int numSamples = 1000; // Numero de muestras a tomar para eliminar el offset

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
    }
    else{
      Serial.println("El sensor no paso el self test, revise el sensor");
    }  
  
  MPU6050Offsets();

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
