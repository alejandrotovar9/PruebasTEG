#include <accl.h>
#include <SPI.h>
#include <SD.h>

#define ARCHIVO_SD "/pruebaSD_TEG.txt"

const int CS = 5;

void copiarCSVenSD(void *pvParameters);
void setup_SD();