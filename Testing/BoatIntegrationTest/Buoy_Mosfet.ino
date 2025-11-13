// ------------------- BUOY MODULE (Heltec Wireless Tracker / ESP32) -------------------
// - 12 V siren switched by N-MOSFET driver (e.g., Duinotech XC4488) from a GPIO
// - LoRa receives boat beacons ("id,lat,lon"); optional buoy self-beacon
// - Proximity alarm within BUOY_ALERT_RANGE triggers siren (with hysteresis)
// - Test profiles let you force RX-only / TX-only / Siren-only / TX+Siren for current tests

#include "LoRaWan_APP.h"
#include "HT_TinyGPS++.h"
#include "Arduino.h"

// ===================== Identity =====================
#define NODE_ID 201          // 200+ reserved for buoys

// ===================== Options =====================
#define BUOY_TRANSMIT 1      // 0 = listen-only, 1 = also beacon own position

// --------------------- TEST PROFILES ---------------------
// 0 = Normal (distance-based siren + optional TX)
// 1 = RX-only (radio listen), no TX, siren OFF
// 2 = TX-only (no siren), fixed cadence for power testing
// 3 = Siren-only fixed pattern (25% duty), no TX
// 4 = TX + Siren fixed pattern (worst-case average power test)
#define TEST_PROFILE 0

// ===================== Pins (Heltec Wireless Tracker) =====================
// Siren MOSFET DRIVER input (active HIGH on most modules; invert in code if needed)
#define SIREN_PIN         7

// GNSS
#define VGNSS_CTRL        3          // GNSS power control
#define GPS_STATUS_LED    45         // show fix
// GNSS UART on Serial1 (confirm your board’s silk: RX=33, TX=34)
#define GNSS_RX           33
#define GNSS_TX           34
#define GNSS_BAUD         9600       // many Heltec GNSS default to 9600. Try 115200 if needed.

// ===================== LoRa Config (AU915 example) =====================
#define RF_FREQUENCY              915000000
#define TX_OUTPUT_POWER           20
#define LORA_BANDWIDTH            1     // 0=125kHz, 1=250kHz, 2=500kHz
#define LORA_SPREADING_FACTOR     7
#define LORA_CODINGRATE           1
#define LORA_PREAMBLE_LENGTH      8
#define LORA_SYMBOL_TIMEOUT       0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON      false
#define BUFFER_SIZE               255

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];
static RadioEvents_t RadioEvents;

// ===================== Scheduling =====================
const unsigned long baseInterval = 2000;  // TX period in ms (used when BUOY_TRANSMIT or TEST_PROFILE 2/4)
unsigned long nextTx = 0;
void scheduleNextTx() {
  // small deterministic offset by node ID to reduce collision chance in tests
  unsigned long offset = (NODE_ID % 10) * 200; // 0–1800 ms
  nextTx = millis() + baseInterval + offset;
}

// ===================== GPS =====================
TinyGPSPlus GPS;
double my_lat = 0.0, my_lon = 0.0;
bool gps_fix = false;

void gpsConfig() {
  pinMode(VGNSS_CTRL, OUTPUT);
  digitalWrite(VGNSS_CTRL, HIGH);            // power GNSS on
  Serial1.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX, GNSS_TX);

  pinMode(GPS_STATUS_LED, OUTPUT);
  digitalWrite(GPS_STATUS_LED, LOW);
}

void gpsReadNonBlocking() {
  while (Serial1.available() > 0) {
    GPS.encode(Serial1.read());
  }
  if (GPS.location.isUpdated()) {
    my_lat = GPS.location.lat();
    my_lon = GPS.location.lng();
    gps_fix = GPS.location.isValid();
    digitalWrite(GPS_STATUS_LED, gps_fix ? HIGH : LOW);
    if (gps_fix) {
      Serial.printf("[GPS] fix: %.6f, %.6f\n", my_lat, my_lon);
    }
  }
}

// ===================== Siren (MOSFET-switched) =====================
void sirenInit() {
  pinMode(SIREN_PIN, OUTPUT);
  digitalWrite(SIREN_PIN, LOW);  // OFF at boot
}

inline void sirenOn()  { digitalWrite(SIREN_PIN, HIGH); } // if your driver inverts, swap HIGH/LOW
inline void sirenOff() { digitalWrite(SIREN_PIN, LOW);  }

// Fixed, repeatable pattern for current testing: 1 s ON / 4 s period (25% duty)
void sirenPatternTask() {
  const unsigned long period = 4000, onTime = 1000;
  unsigned long t = millis() % period;
  (t < onTime) ? sirenOn() : sirenOff();
}

// ===================== Fleet Table (boats heard) =====================
struct NodeData {
  int id;
  double lat;
  double lon;
  unsigned long lastUpdate;
};

#define MAX_NODES 30
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
  if (nodeCount < MAX_NODES) {
    nodeTable[nodeCount++] = { id, lat, lon, millis() };
  }
}

void pruneStaleNodes(unsigned long ttl_ms = 15000) {
  unsigned long now = millis();
  int w = 0;
  for (int i = 0; i < nodeCount; i++) {
    if (now - nodeTable[i].lastUpdate <= ttl_ms) {
      if (w != i) nodeTable[w] = nodeTable[i];
      w++;
    }
  }
  if (w != nodeCount) Serial.printf("[Table] Pruned %d stale nodes\n", nodeCount - w);
  nodeCount = w;
}

// Expect "id,lat,lon"
void parseSingleRecord(const char *msg) {
  int id; double lat, lon;
  if (sscanf(msg, "%d,%lf,%lf", &id, &lat, &lon) == 3) {
    if (id != NODE_ID) updateNode(id, lat, lon);
  }
}

// ===================== Proximity Alarm =====================
#define BUOY_ALERT_RANGE   50.0   // meters
#define HYSTERESIS          2.0   // meters
static bool sirenState = false;

void checkDistances() {
  bool alert = false;

  for (int i = 0; i < nodeCount; i++) {
    // Assume boats use IDs < 200
    if (nodeTable[i].id >= 200) continue;

    double dist = TinyGPSPlus::distanceBetween(
      my_lat, my_lon, nodeTable[i].lat, nodeTable[i].lon);

    if (dist < BUOY_ALERT_RANGE) {
      alert = true;
      break;
    }
  }

  if (alert && !sirenState) {
    sirenOn();  sirenState = true;  Serial.println("[ALARM] Siren ON (proximity)");
  } else if (!alert && sirenState) {
    bool stillClose = false;
    for (int i = 0; i < nodeCount; i++) {
      if (nodeTable[i].id >= 200) continue;
      double dist = TinyGPSPlus::distanceBetween(
        my_lat, my_lon, nodeTable[i].lat, nodeTable[i].lon);
      if (dist < BUOY_ALERT_RANGE + HYSTERESIS) { stillClose = true; break; }
    }
    if (!stillClose) {
      sirenOff(); sirenState = false; Serial.println("[ALARM] Siren OFF (hysteresis clear)");
    }
  }
}

// ===================== LoRa Callbacks =====================
void OnTxDone(void) {
  // After TX, go back to continuous RX
  Radio.Rx(0);
}
void OnTxTimeout(void) {
  Serial.println("[LORA] TX timeout → back to RX");
  Radio.Rx(0);
}
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  memcpy(rxpacket, payload, size);
  rxpacket[size] = '\0';
  Radio.Sleep();

  // Packet: "id,lat,lon" from boats
  parseSingleRecord(rxpacket);
  // Resume RX
  Radio.Rx(0);
}

// ===================== LoRa Init =====================
void loraConfig() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  RadioEvents.TxDone   = OnTxDone;
  RadioEvents.TxTimeout= OnTxTimeout;
  RadioEvents.RxDone   = OnRxDone;

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

1234567891011121314151617181920
#define LORA_BANDWIDTH 0 // 125 kHz
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 1
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define BUFFER_SIZE 255

char txpacket[BUFFER_SIZE];


void sirenBeepSequence() {
  for (int i = 0; i < 3; i++) {
    sirenOn();
    delay(150);
    sirenOff();
    delay(150);
  }
}

// ===================== Setup / Loop =====================
void setup() {
  loraConfig();
  gpsConfig();
  sirenInit();

  // --- Startup confirmation beeps ---
  Serial.println("[BOOT] Performing startup beep test...");
  sirenBeepSequence();

  scheduleNextTx();
  Serial.printf("[BOOT] BUOY NODE_ID=%d, TEST_PROFILE=%d\n", NODE_ID, TEST_PROFILE);
}

void loop() {
  // Keep GNSS fed without blocking
  gpsReadNonBlocking();

  // ----- TEST PROFILE BEHAVIOUR -----
  #if (TEST_PROFILE == 1)
    // RX-only (radio listening), no TX, siren OFF
    sirenOff();

  #elif (TEST_PROFILE == 2)
    // TX-only (fixed cadence)
    if (millis() >= nextTx) {
      if (gps_fix)
        snprintf(txpacket, sizeof(txpacket), "%d,%.6f,%.6f", NODE_ID, my_lat, my_lon);
      else
        snprintf(txpacket, sizeof(txpacket), "%d,0.000000,0.000000", NODE_ID);
      Serial.printf("[TX] %s\n", txpacket);
      Radio.Send((uint8_t*)txpacket, strlen(txpacket));
      scheduleNextTx();
    }
    sirenOff();

  #elif (TEST_PROFILE == 3)
    // Siren-only fixed pattern (no TX)
    sirenPatternTask();

  #elif (TEST_PROFILE == 4)
    // TX + Siren fixed pattern (worst-case average)
    if (millis() >= nextTx) {
      if (gps_fix)
        snprintf(txpacket, sizeof(txpacket), "%d,%.6f,%.6f", NODE_ID, my_lat, my_lon);
      else
        snprintf(txpacket, sizeof(txpacket), "%d,0.000000,0.000000", NODE_ID);
      Serial.printf("[TX] %s\n", txpacket);
      Radio.Send((uint8_t*)txpacket, strlen(txpacket));
      scheduleNextTx();
    }
    sirenPatternTask();

  #else
    // ----- NORMAL MODE -----
    // Optional buoy beacon
    #if BUOY_TRANSMIT
    if (millis() >= nextTx) {
      if (gps_fix)
        snprintf(txpacket, sizeof(txpacket), "%d,%.6f,%.6f", NODE_ID, my_lat, my_lon);
      else
        snprintf(txpacket, sizeof(txpacket), "%d,0.000000,0.000000", NODE_ID);
      Serial.printf("[TX] %s\n", txpacket);
      Radio.Send((uint8_t*)txpacket, strlen(txpacket));
      scheduleNextTx();
    }
    #endif

    // Only compute proximity alarm if we know our own position
    if (gps_fix) {
      checkDistances();
    } else {
      // If we lost fix, ensure siren is off to avoid false alerts
      sirenOff(); sirenState = false;
    }
  #endif

  // Service radio IRQs
  Radio.IrqProcess();

  // Keep the table tidy
  pruneStaleNodes();
}
