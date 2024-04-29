#include <tasks_queues.h>
#include <handles.h>
#include <lora_header.h>
#include <mqtt_header.h>

TaskHandle_t xHandle_blink;

//INICIALIZCION DE OBJETOS
ESP32Time rtc;
const char *ntpServer = "0.north-america.pool.ntp.org"; // 0.north-america.pool.ntp.org FUNCIONA CON RED RAPIDA
/*
0.north-america.pool.ntp.org
time.google.com - Google's NTP server.
time.windows.com - Microsoft's NTP server.
time.nist.gov - NIST's NTP server.
time.apple.com - Apple's NTP server.
time.cloudflare.com - Cloudflare's NTP server
*/
// CON REDES LENTAS NO FUNCIONA BIEN

/*Caracas, Venezuela está en la zona horaria GMT-4 */
const long gmtOffset_sec = -14400; //-4*60*60
const int daylightOffset_sec = 0;

void IRAM_ATTR ISR_button()
{
  transmitFlag = true;
  vTaskResume(xHandle_send_task);
}

void blink(void *pvParameters)
{
  while (1)
  {
    // Blink an LED on PIN 12
    digitalWrite(12, HIGH);
    delay(1000);
    digitalWrite(12, LOW);
    delay(1000);
  }
}


void setup()
{
  Serial.begin(115200);

  //Crear cola para enviar estructura de tipo BufferACL
  xQueueBufferACL = xQueueCreate(3, sizeof(BufferACL));
  xQueueTempHumInc = xQueueCreate(1, sizeof(THIPacket));


  pinMode(12, OUTPUT);

  pinMode(22, OUTPUT);

  pinMode(32, INPUT);
  attachInterrupt(digitalPinToInterrupt(32), ISR_button, RISING);

  // Pull down for pin 2
  // pinMode(2, INPUT_PULLDOWN);

  setup_wifi();

  setup_mqtt();

  // Init and get the time
  /*---------set with NTP---------------*/
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  struct tm timeinfo;

  if (getLocalTime(&timeinfo))
  {
    rtc.setTimeStruct(timeinfo);
  }

  update_timepacket();
  // printLocalTime(); //Imprime hora actual tras actualizacion por NTP

  // configTime(gmtOffset_sec, daylightOffset_sec, ntpServer, "time.nist.gov", "time.google.com");
  // printLocalTime();

  setup_lora();

  xTaskCreatePinnedToCore(
      send_task,          /* Function to implement the task */
      "send_task",        /* Name of the task */
      1024 * 4,           /* Stack size in words */
      NULL,               /* Task input parameter */
      1,                  /* Priority of the task */
      &xHandle_send_task, /* Task handle. */
      0);                 /* Core where the task should run */

  vTaskSuspend(xHandle_send_task);

  xTaskCreatePinnedToCore(
      send_RTC_task,          /* Function to implement the task */
      "send_RTC_task",        /* Name of the task */
      1024 * 2,               /* Stack size in words */
      NULL,                   /* Task input parameter */
      1,                      /* Priority of the task */
      &xHandle_send_RTC_task, /* Task handle. */
      0);                     /* Core where the task should run */

  vTaskSuspend(xHandle_send_RTC_task);

  xTaskCreatePinnedToCore(
      receive_task,          /* Function to implement the task */
      "receive_task",        /* Name of the task */
      1024 * 4,              /* Stack size in words */
      NULL,                  /* Task input parameter */
      6,                     /* Priority of the task */
      &xHandle_receive_task, /* Task handle. */
      0);                    /* Core where the task should run */

  vTaskSuspend(xHandle_receive_task);

  xTaskCreatePinnedToCore(
      send_mqtt,          /* Function to implement the task */
      "send_mqtt",        /* Name of the task */
      1024 * 12,              /* Stack size in words */
      NULL,                  /* Task input parameter */
      4,                     /* Priority of the task */
      &xHandle_send_mqtt, /* Task handle. */
      0);                    /* Core where the task should run */

  vTaskSuspend(xHandle_send_mqtt);

  xTaskCreatePinnedToCore(
      send_mqtt_thi,          /* Function to implement the task */
      "send_mqtt_thi",        /* Name of the task */
      1024 * 4,              /* Stack size in words */
      NULL,                  /* Task input parameter */
      4,                     /* Priority of the task */
      &xHandle_send_mqtt_thi, /* Task handle. */
      0);                    /* Core where the task should run */

  vTaskSuspend(xHandle_send_mqtt_thi);


  xTaskCreatePinnedToCore(
      blink,          /* Function to implement the task */
      "receive_task", /* Name of the task */
      1024,           /* Stack size in words */
      NULL,           /* Task input parameter */
      0,              /* Priority of the task */
      &xHandle_blink, /* Task handle. */
      0);             /* Core where the task should run */
}

void loop()
{
 delay(1000);
}
