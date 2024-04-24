#include <tasks_queues.h>

#define SIZE_OF_FLOAT_ARRAY 1024
#define CHUNK_SIZE 32

#define NUM_DATOS 1024
#define MEAN_INTERVAL 5
#define NUM_DATOS_TEMP 200
#define NUM_DATOS_INC 800

// Estructuras de datos
struct BufferTempHumedad
{
  float buffertemp[NUM_DATOS_TEMP / 5 - 1] = {};
  float bufferhum[NUM_DATOS_TEMP / 5 - 1] = {};
};

// Estructura que contiene 3 arreglos de 5000 flotantes
struct BufferACL
{
  //long int buffer_timestamp[NUM_DATOS - 1] = {};
  float bufferX[NUM_DATOS] = {};
  float bufferY[NUM_DATOS] = {};
  float bufferZ[NUM_DATOS] = {};
};

struct BufferInclinacion
{
  float bufferRoll[NUM_DATOS_INC / 5 - 1] = {};
  float bufferPitch[NUM_DATOS_INC / 5 - 1] = {};
  float bufferYaw[NUM_DATOS_INC / 5 - 1] = {};
};

struct Packet
{
  byte messageID;
  byte senderID;
  byte receiverID;
  byte payload[128];
};

struct Packet2
{
  byte messageID;
  byte senderID;
  byte receiverID;
  byte payload[1]; // Adjust the size as needed
};

struct TimePacket
{
  byte messageID;
  byte senderID;
  byte receiverID;
  byte payload[24]; // Adjust the size as needed
};

struct timestruct
{
  int year;
  int month;
  int day;
  int hour;
  int minute;
  int second;
};

struct StringPacket
{
  byte messageID;   // Message ID
  byte senderID;
  byte receiverID;
  char payload[19]; // Message (18 characters for "Smart Sensor Ready" + 1 for the null-terminating character)
};

union PacketUnion
{
  Packet packet1;
  Packet2 packet2;
  TimePacket timePacket;
  StringPacket stringPacket;
};

extern bool transmitFlag;


//Prototipos de funciones
void fillBuffer(float *buffer, unsigned char *byteArray, int size, bool corrupted_data_flag);
int leer_datos(size_t packetSize, byte incomingMsgId, byte sender, byte recipient, byte *incoming);
void receive_task(void *pvParameter);
int sendmessage_radiolib(size_t size_data, byte data[]);
void update_timepacket(void);
void send_RTC_task(void *pvParameters);
void send_task(void *pvParameters);
void setup_lora(void);

