#include "LoRaWan_APP.h"
#include "HT_TinyGPS++.h"
#include "Arduino.h"

// ------------- Node Identity (Buoy only) -------------
#define NODE_ID 201 // 200+ range for buoys

// ------------- Pin Config -------------
#define BUZZER_PIN 7
#define GPS_STATUS_LED 45

// ------------- LoRa Config -------------
#define RF_FREQUENCY 915000000
#define TX_OUTPUT_POWER 20
#define LORA_BANDWIDTH 1 // 125 kHz
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 1
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define BUFFER_SIZE 255

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;

// ------------- Scheduling -------------
const unsigned long baseInterval = 2000; // 2 s cycle
unsigned long nextTx = 0;

void scheduleNextTx()
{
  unsigned long offset = (NODE_ID % 10) * 200; // 0–1800 ms spread
  nextTx = millis() + baseInterval + offset;
}

// ------------- GPS -------------
TinyGPSPlus GPS;
#define VGNSS_CTRL 3

double my_lat = 0.0;
double my_lon = 0.0;
bool gps_fix = false;

void gpsConfig()
{
  pinMode(VGNSS_CTRL, OUTPUT);
  digitalWrite(VGNSS_CTRL, HIGH);
  Serial1.begin(115200, SERIAL_8N1, 33, 34);

  pinMode(GPS_STATUS_LED, OUTPUT);
  digitalWrite(GPS_STATUS_LED, LOW);
}

void gpsReadNonBlocking()
{
  while (Serial1.available() > 0)
  {
    char c = Serial1.read();
    GPS.encode(c);
  }
  if (GPS.location.isUpdated())
  {
    my_lat = GPS.location.lat();
    my_lon = GPS.location.lng();
    gps_fix = true;
    Serial.printf("GPS fix: %.6f, %.6f\n", my_lat, my_lon);
    digitalWrite(GPS_STATUS_LED, HIGH);
  }
}

// ------------- Buzzer -------------

void buzzerInit()
{
  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);
}

void buzzOn()
{
  tone(BUZZER_PIN, 1000);
  Serial.println("Buzzer ON");
}
void buzzOff()
{
  noTone(BUZZER_PIN);
  Serial.println("Buzzer OFF");
}

// ------------- Fleet Table -------------
struct NodeData
{
  int id;
  double lat;
  double lon;
  unsigned long lastUpdate;
};

#define MAX_NODES 30
NodeData nodeTable[MAX_NODES];
int nodeCount = 0;

void updateNode(int id, double lat, double lon)
{
  for (int i = 0; i < nodeCount; i++)
  {
    if (nodeTable[i].id == id)
    {
      nodeTable[i].lat = lat;
      nodeTable[i].lon = lon;
      nodeTable[i].lastUpdate = millis();
      return;
    }
  }
  if (nodeCount < MAX_NODES)
  {
    nodeTable[nodeCount++] = {id, lat, lon, millis()};
  }
}

void parseBulk(const char *bulk)
{
  char buf[BUFFER_SIZE];
  strncpy(buf, bulk, sizeof(buf));
  buf[sizeof(buf) - 1] = '\0';

  char *entry = strtok(buf, ";");
  while (entry != NULL)
  {
    int id;
    double lat, lon;
    if (sscanf(entry, "%d,%lf,%lf", &id, &lat, &lon) == 3)
    {
      if (id != NODE_ID)
        updateNode(id, lat, lon);
    }
    entry = strtok(NULL, ";");
  }
}

// ------------- LoRa Callbacks -------------
void OnTxDone(void)
{
  Radio.Rx(0);
}

void OnTxTimeout(void)
{
  Serial.println("TX timeout → back to RX");
  Radio.Rx(0);
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
  memcpy(rxpacket, payload, size);
  rxpacket[size] = '\0';
  Radio.Sleep();

  // Buoy expects CENTRAL bulk messages: "id,lat,lon;id,lat,lon;..."
  parseBulk(rxpacket);

  Radio.Rx(0); // return to RX after handling
}

// ------------- Distance Checks (Buoy) -------------
#define BUOY_ALERT_RANGE 50.0
#define HYSTERESIS 2.0
static bool buzzerState = false;

void checkDistances()
{
  bool alert = false;

  for (int i = 0; i < nodeCount; i++)
  {
    // Boats are < 200 by convention
    if (nodeTable[i].id >= 200)
      continue;

    double dist = TinyGPSPlus::distanceBetween(
        my_lat, my_lon,
        nodeTable[i].lat, nodeTable[i].lon);

    if (dist < BUOY_ALERT_RANGE)
    {
      alert = true;
      break;
    }
  }

  if (alert && !buzzerState)
  {
    buzzOn();
    buzzerState = true;
  }
  else if (!alert && buzzerState)
  {
    bool stillClose = false;
    for (int i = 0; i < nodeCount; i++)
    {
      if (nodeTable[i].id >= 200)
        continue;
      double dist = TinyGPSPlus::distanceBetween(
          my_lat, my_lon,
          nodeTable[i].lat, nodeTable[i].lon);
      if (dist < BUOY_ALERT_RANGE + HYSTERESIS)
      {
        stillClose = true;
        break;
      }
    }
    if (!stillClose)
    {
      buzzOff();
      buzzerState = false;
    }
  }
}

// ------------- Setup / LoRa init -------------
void loraConfig()
{
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone = OnRxDone;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  Radio.Rx(0);
}

void setup()
{
  loraConfig();
  gpsConfig();
  buzzerInit();
  scheduleNextTx();
}

// ------------- Main Loop (Buoy) -------------
void loop()
{
  gpsReadNonBlocking();

  // Periodically TX our own position
  if (millis() >= nextTx)
  {
    if (gps_fix)
    {
      snprintf(txpacket, sizeof(txpacket),
               "%d,%.6f,%.6f", NODE_ID, my_lat, my_lon);
    }
    else
    {
      snprintf(txpacket, sizeof(txpacket),
               "%d,0.000000,0.000000", NODE_ID);
    }
    Serial.printf("TX: %s\n", txpacket);
    Radio.Send((uint8_t *)txpacket, strlen(txpacket));
    scheduleNextTx();
  }

  // Always service IRQs
  Radio.IrqProcess();

  // Proximity alarm only when we have our own fix
  if (gps_fix)
  {
    checkDistances();
  }
}
