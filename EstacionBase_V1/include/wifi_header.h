#include <WiFi.h>

#ifndef WIFI_CREDENTIALS_H
#define WIFI_CREDENTIALS_H

//Parametros para conexion WiFi
// const char* ssid     = "JET Home";
// const char* password = "chemundo300665";

extern const char* ssid;
extern const char* password;

// const char* ssid     = "lide";
// const char* password = "eie-lide*";

// const char* ssid     = "decc";
// const char* password = "MelchorCentenoVallenilla";

#endif // WIFI_CREDENTIALS_H

//Prototipos de funciones
void setup_wifi(void);