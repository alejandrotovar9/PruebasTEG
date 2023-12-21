#include <sd_storage.h>

File myFile;
char bufferx[5000];

extern QueueHandle_t bufferQueue;

void setup_sd(){

  while (!Serial) { ; }  // wait for serial port to connect. Needed for native USB port only
    Serial.println("Initializing SD card...");
    if (!SD.begin(CS)) {
        Serial.println("initialization failed!");
        return;
  }
  Serial.println("initialization done.");
  /*Headers for CSV*/
  myFile = SD.open(ARCHIVO_SD, FILE_WRITE);
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
    Serial.println(ARCHIVO_SD);
  }
}

void copiarCSVenSD(void *pvParameters){

  BufferACL datos_buffer_sd;

  if(xQueueReceive(bufferQueue, &datos_buffer_sd, portMAX_DELAY)){
    myFile = SD.open(ARCHIVO_SD, FILE_WRITE);
    for (int i = 0; i < NUM_DATOS; i++) {
      //myFile = SD.open(ARCHIVO_SD, FILE_WRITE);

      //Serial.println(a.acceleration.x);
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
  }
  //myFile.print(bufferx);
  //myFile.close(); // close the file:
}
