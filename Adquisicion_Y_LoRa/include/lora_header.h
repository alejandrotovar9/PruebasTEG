#include <tasks_queues.h>

#define SIZE_OF_FLOAT_ARRAY 2048
#define CHUNK_SIZE 32
//Son 32 chunks, de 32 floats cada uno, para un total de 1024 floats

//volatile bool sx1278Interrupt = false;

//Prototipos de funciones
void poll_packet(void *pvParameters);
void send_packet(void *pvParameters);
void poll_modo_operacion(void *pvParameters);
void setupLoRa(void);
