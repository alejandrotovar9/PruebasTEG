#include <tasks_queues.h>

//Prototype functions
void setup_mqtt();
void send_mqtt(void *pvParameters);
void keepalive_task(void *pvParameters);
