#include "LoRaWan_APP.h"
#include "HT_TinyGPS++.h"
#include "Arduino.h"

// -------- Boat Identity --------
#define NODE_ID 101   // 100+ range for boats

// -------- LoRa Config --------
#define RF_FREQUENCY 915000000
#define TX_OUTPUT_POWER 20
#define LORA_BANDWIDTH 0 // 125 kHz
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

// -------- Scheduling --------
const unsigned long baseInterval = 5000; // 5 s
unsigned long nextTx = 0;
void scheduleNextTx() {
  unsigned long offset = (NODE_ID % 10) * 200; // 0–1800 ms spread
  nextTx = millis() + baseInterval + offset;
}

// -------- GPS --------
TinyGPSPlus GPS;
#define VGNSS_CTRL 3
double my_lat = 0.0, my_lon = 0.0;
bool gps_fix = false;

void gpsConfig() {
  pinMode(VGNSS_CTRL, OUTPUT);
  digitalWrite(VGNSS_CTRL, HIGH);
  // GNSS on Serial1 (RX=33, TX=34)
  Serial1.begin(115200, SERIAL_8N1, 33, 34);
}

void gpsReadNonBlocking() {
  while (Serial1.available() > 0) {
    GPS.encode(Serial1.read());
  }
  if (GPS.location.isUpdated()) {
    my_lat = GPS.location.lat();
    my_lon = GPS.location.lng();
    gps_fix = true;
    Serial.printf("GPS fix: %.6f, %.6f\n", my_lat, my_lon);
  }
}

// -------- Buzzer --------
#define BUZZER_PIN 7
void buzzerInit() { pinMode(BUZZER_PIN, OUTPUT); noTone(BUZZER_PIN); }
void buzzOn()  { tone(BUZZER_PIN, 1000); Serial.println("Buzzer ON"); }
void buzzOff() { noTone(BUZZER_PIN);     Serial.println("Buzzer OFF"); }

// -------- Fleet Table --------
struct NodeData { int id; double lat; double lon; unsigned long lastUpdate; };
#define MAX_NODES 60
NodeData nodeTable[MAX_NODES];
int nodeCount = 0;

void updateNode(int id, double lat, double lon) {
  for (int i = 0; i < nodeCount; i++) {
    if (nodeTable[i].id == id) {
      nodeTable[i].lat = lat;
      nodeTable[i].lon = lon;
      nodeTable[i].lastUpdate = millis();
      return;
    }
  }
  if (nodeCount < MAX_NODES) nodeTable[nodeCount++] = { id, lat, lon, millis() };
}

// CENTRAL bulk: "id,lat,lon;id,lat,lon;..."
void parseBulk(const char *bulk) {
  char buf[BUFFER_SIZE];
  strncpy(buf, bulk, sizeof(buf));
  buf[sizeof(buf) - 1] = '\0';
  for (char *entry = strtok(buf, ";"); entry != NULL; entry = strtok(NULL, ";")) {
    int id; double lat, lon;
    if (sscanf(entry, "%d,%lf,%lf", &id, &lat, &lon) == 3) {
      if (id != NODE_ID) updateNode(id, lat, lon);
    }
  }
}

// -------- LoRa Callbacks --------
void OnTxDone(void)         { Radio.Rx(0); }
void OnTxTimeout(void)      { Serial.println("TX timeout → back to RX"); Radio.Rx(0); }
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  memcpy(rxpacket, payload, size);
  rxpacket[size] = '\0';
  Radio.Sleep();
  parseBulk(rxpacket);      // Boat expects CENTRAL bulk updates
  Radio.Rx(0);
}

// -------- Proximity (Boat vs other boats) --------
#define BOAT_ALERT_RANGE 25.0
#define HYSTERESIS 2.0
static bool buzzerState = false;

void checkDistances() {
  bool alert = false;
  for (int i = 0; i < nodeCount; i++) {
    // Other boats are <200 and not self
    if (nodeTable[i].id >= 200 || nodeTable[i].id == NODE_ID) continue;
    double dist = TinyGPSPlus::distanceBetween(my_lat, my_lon, nodeTable[i].lat, nodeTable[i].lon);
    if (dist < BOAT_ALERT_RANGE) { alert = true; break; }
  }

  if (alert && !buzzerState) {
    buzzOn(); buzzerState = true;
  } else if (!alert && buzzerState) {
    bool stillClose = false;
    for (int i = 0; i < nodeCount; i++) {
      if (nodeTable[i].id >= 200 || nodeTable[i].id == NODE_ID) continue;
      double dist = TinyGPSPlus::distanceBetween(my_lat, my_lon, nodeTable[i].lat, nodeTable[i].lon);
      if (dist < BOAT_ALERT_RANGE + HYSTERESIS) { stillClose = true; break; }
    }
    if (!stillClose) { buzzOff(); buzzerState = false; }
  }
}

// -------- LoRa init --------
void loraConfig() {
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

// -------- Setup / Loop --------
void setup() {
  loraConfig();
  gpsConfig();
  buzzerInit();
  scheduleNextTx();
}

void loop() {
  gpsReadNonBlocking();

  // Periodic own-position beacon (for CENTRAL to ingest)
  if (millis() >= nextTx) {
    if (gps_fix) snprintf(txpacket, sizeof(txpacket), "%d,%.6f,%.6f", NODE_ID, my_lat, my_lon);
    else         snprintf(txpacket, sizeof(txpacket), "%d,0.000000,0.000000", NODE_ID);
    Serial.printf("TX: %s\n", txpacket);
    Radio.Send((uint8_t *)txpacket, strlen(txpacket));
    scheduleNextTx();
  }

  Radio.IrqProcess();

  if (gps_fix) checkDistances();
}
