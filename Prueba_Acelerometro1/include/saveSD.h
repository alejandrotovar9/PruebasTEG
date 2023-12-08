#include <accl.h>
#include <SPI.h>
#include <SD.h>

#define ARCHIVO_SD "/pruebaSD.txt"

const int CS = 5;

bool copiarCSVenSD (int num_datos);
bool setupSD();