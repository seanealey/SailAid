#include "LoRaWan_APP.h"
#include "HT_TinyGPS++.h"
#include "Arduino.h"

// ------------- Central Identity -------------
#define CENTRAL_ID 301

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
unsigned long nextCentralTx = 0;

void scheduleNextCentralTx()
{
  nextCentralTx = millis() + baseInterval / 2; // mid-cycle broadcast
}

// ------------- GPS (Central's own position) -------------
TinyGPSPlus GPS;
#define VGNSS_CTRL 3

double my_lat = 0.0;
double my_lon = 0.0;
bool gps_fix = false;

void gpsConfig()
{
  pinMode(VGNSS_CTRL, OUTPUT);
  digitalWrite(VGNSS_CTRL, HIGH);
  // RX=33, TX=34 (to GNSS module)
  Serial1.begin(115200, SERIAL_8N1, 33, 34);
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
    // Central usually stays quiet; uncomment if you want logs:
    // Serial.printf("CENTRAL GPS: %.6f, %.6f\n", my_lat, my_lon);
  }
}

// ------------- Fleet Table -------------
struct NodeData
{
  int id;
  double lat;
  double lon;
  unsigned long lastUpdate;
};

#define MAX_NODES 60 // room for many boats/buoys
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

// ------------- Central Bulk TX -------------
void sendBulkList()
{
  char bulk[BUFFER_SIZE];
  bulk[0] = '\0';

  // Start with CENTRAL's position
  char entry[40];
  snprintf(entry, sizeof(entry),
           "%d,%.6f,%.6f;",
           CENTRAL_ID, my_lat, my_lon);
  strncat(bulk, entry, sizeof(bulk) - strlen(bulk) - 1);

  // Add all known nodes
  for (int i = 0; i < nodeCount; i++)
  {
    snprintf(entry, sizeof(entry),
             "%d,%.6f,%.6f;",
             nodeTable[i].id,
             nodeTable[i].lat,
             nodeTable[i].lon);
    strncat(bulk, entry, sizeof(bulk) - strlen(bulk) - 1);
  }

  Radio.Send((uint8_t *)bulk, strlen(bulk));

  // Also print lines for your visualiser
  Serial.printf("FLEET,%d,%.6f,%.6f\n", CENTRAL_ID, my_lat, my_lon);
  for (int i = 0; i < nodeCount; i++)
  {
    Serial.printf("FLEET,%d,%.6f,%.6f\n",
                  nodeTable[i].id,
                  nodeTable[i].lat,
                  nodeTable[i].lon);
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

  // Expect single-node beacons: "id,lat,lon"
  int id;
  double lat, lon;
  if (sscanf(rxpacket, "%d,%lf,%lf", &id, &lat, &lon) == 3)
  {
    updateNode(id, lat, lon);
    // Serial.printf("RX (%d): %.6f, %.6f (RSSI %d)\n", id, lat, lon, rssi);
  }
  else
  {
    Serial.println("CENTRAL parse failed");
  }

  Radio.Rx(0); // return to RX after handling
}

// ------------- LoRa init -------------
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

// ------------- Setup / Loop -------------
void setup()
{
  loraConfig();
  gpsConfig();
  scheduleNextCentralTx();
}

void loop()
{
  gpsReadNonBlocking();

  if (millis() >= nextCentralTx)
  {
    sendBulkList();
    scheduleNextCentralTx();
  }

  Radio.IrqProcess(); // keep radio responsive
}
