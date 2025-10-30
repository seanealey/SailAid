#include "LoRaWan_APP.h"
#include "HT_TinyGPS++.h"
#include "Arduino.h"

// ================== Node Identity ==================
#define NODE_ID 101          // 100+ range for boats
#define TARGET_BUOY_ID 201   // Fixed buoy ID to navigate toward

// ================== Pins ==================
#define VGNSS_CTRL 3
#define GPS_RX 33
#define GPS_TX 34
#define BUZZER_PIN 7 // pin used for testing, can be used for the speaker

// ================== LoRa Config ==================
#define RF_FREQUENCY 915000000
#define TX_OUTPUT_POWER 20
#define LORA_BANDWIDTH 1  // 250kz
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 1
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define BUFFER_SIZE 255

static RadioEvents_t RadioEvents;
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

// ================== Scheduling ==================
const unsigned long baseIntervalMs = 2000;
unsigned long nextTx = 0;
void scheduleNextTx() {
  unsigned long offset = (NODE_ID % 10) * 200; // 0–1800 ms spread
  nextTx = millis() + baseIntervalMs + offset;
}

// ================== GPS ==================
TinyGPSPlus GPS;
volatile double boat_lat = 0.0, boat_lon = 0.0;
volatile bool boat_has_fix = false;

// ================== Buoy Data ==================
struct BuoyData {
  int id;
  double lat;
  double lon;
  unsigned long lastUpdate;
};
volatile BuoyData targetBuoy = {TARGET_BUOY_ID, 0.0, 0.0, 0};
const unsigned long BUOY_TTL_MS = 15000;

// ================== Concurrency ==================
SemaphoreHandle_t stateMutex;

// ================== Placeholders ==================
void computeBearing(double boatLat, double boatLon,
                               double buoyLat, double buoyLon) {
  // Throttle prints so we don't spam the serial monitor (~2 Hz)
  static unsigned long lastPrintMs = 0;
  const unsigned long printIntervalMs = 500;
  unsigned long now = millis();
  if (now - lastPrintMs < printIntervalMs) return;
  lastPrintMs = now;

  // Distance in meters
  double dist_m = TinyGPSPlus::distanceBetween(boatLat, boatLon, buoyLat, buoyLon);

  // Bearing in degrees (0..360, 0 = North)
  double bearing_deg = TinyGPSPlus::courseTo(boatLat, boatLon, buoyLat, buoyLon);

  // Cardinal helper
  auto toCardinal = [](double deg)->const char* {
    static const char* dirs[] = {"N","NNE","NE","ENE","E","ESE","SE","SSE",
                                 "S","SSW","SW","WSW","W","WNW","NW","NNW","N"};
    int idx = (int)round((deg / 22.5));
    if (idx < 0) idx = 0;
    if (idx > 16) idx = 16;
    return dirs[idx];
  };

  Serial.printf("NAV → Dist: %.1f m | Bearing: %.1f° (%s)\n",
                dist_m, bearing_deg, toCardinal(bearing_deg));
}


// ================== Buzzer ==================
void buzzerInit() { pinMode(BUZZER_PIN, OUTPUT); noTone(BUZZER_PIN); }
void buzzOn()  { tone(BUZZER_PIN, 1000); }
void buzzOff() { noTone(BUZZER_PIN);     }

// ================== RX Parse ==================
void handleRxMessage(const char* msg) {
  int id; double lat, lon;
  if (sscanf(msg, "%d,%lf,%lf", &id, &lat, &lon) == 3) {
    if (id == TARGET_BUOY_ID) {
      if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
        targetBuoy.lat = lat;
        targetBuoy.lon = lon;
        targetBuoy.lastUpdate = millis();
        xSemaphoreGive(stateMutex);
      }
    }
  }
}

// ================== LoRa Callbacks ==================
void OnTxDone(void) { Radio.Rx(0); }
void OnTxTimeout(void) { Radio.Rx(0); }
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  size = min<uint16_t>(size, BUFFER_SIZE - 1);
  memcpy(rxpacket, payload, size);
  rxpacket[size] = '\0';
  Radio.Sleep();
  handleRxMessage(rxpacket);
  Radio.Rx(0);
}

// ================== Tasks ==================
TaskHandle_t gpsTaskHandle = nullptr;
TaskHandle_t loraTaskHandle = nullptr;
TaskHandle_t appTaskHandle = nullptr;

// --- GPS Task ---
void gpsTask(void* pv) {
  for (;;) {
    while (Serial1.available() > 0) {
      GPS.encode(Serial1.read());
    }
    if (GPS.location.isUpdated()) {
      double lat = GPS.location.lat();
      double lon = GPS.location.lng();
      if (xSemaphoreTake(stateMutex, 5 / portTICK_PERIOD_MS) == pdTRUE) {
        boat_lat = lat;
        boat_lon = lon;
        boat_has_fix = true;
        xSemaphoreGive(stateMutex);
      }
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

// --- LoRa Task ---
void loraTask(void* pv) {
  scheduleNextTx();
  for (;;) {
    // Transmit our position
    if (millis() >= nextTx) {
      double lat, lon; bool fix;
      if (xSemaphoreTake(stateMutex, 10 / portTICK_PERIOD_MS) == pdTRUE) {
        lat = boat_lat; lon = boat_lon; fix = boat_has_fix;
        xSemaphoreGive(stateMutex);
      } else { lat = 0.0; lon = 0.0; fix = false; }

      if (fix)
        snprintf(txpacket, sizeof(txpacket), "%d,%.6f,%.6f", NODE_ID, lat, lon);
      else
        snprintf(txpacket, sizeof(txpacket), "%d,0.000000,0.000000", NODE_ID);

      Radio.Send((uint8_t*)txpacket, strlen(txpacket));
      scheduleNextTx();
    }

    Radio.IrqProcess();
    vTaskDelay(2 / portTICK_PERIOD_MS);
  }
}

// --- App Task: bearing placeholder ---
void appTask(void* pv) {
  for (;;) {
    double bLat, bLon, buoyLat, buoyLon;
    bool fix;
    unsigned long buoyAge = 0;

    if (xSemaphoreTake(stateMutex, 10 / portTICK_PERIOD_MS) == pdTRUE) {
      bLat = boat_lat;
      bLon = boat_lon;
      fix = boat_has_fix;
      buoyLat = targetBuoy.lat;
      buoyLon = targetBuoy.lon;
      buoyAge = millis() - targetBuoy.lastUpdate;
      xSemaphoreGive(stateMutex);
    }

    if (fix && buoyAge <= BUOY_TTL_MS && buoyLat != 0.0 && buoyLon != 0.0) {
      computeBearing(bLat, bLon, buoyLat, buoyLon);
    }

    vTaskDelay(50 / portTICK_PERIOD_MS); // ~20 Hz
  }
}

// ================== Init ==================
void gpsConfig() {
  pinMode(VGNSS_CTRL, OUTPUT);
  digitalWrite(VGNSS_CTRL, HIGH);
  Serial1.begin(115200, SERIAL_8N1, GPS_RX, GPS_TX);
}

void loraConfig() {
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

// ================== Arduino Hooks ==================
void setup() {
  Serial.begin(115200);
  gpsConfig();
  loraConfig();
  buzzerInit();

  stateMutex = xSemaphoreCreateMutex();

  xTaskCreate(gpsTask,  "gpsTask",  4096, nullptr, 2, &gpsTaskHandle);
  xTaskCreate(loraTask, "loraTask", 4096, nullptr, 3, &loraTaskHandle);
  xTaskCreate(appTask,  "appTask",  4096, nullptr, 1, &appTaskHandle);

  Serial.printf("Target buoy set to %d\n", TARGET_BUOY_ID);
}

void loop() {
  // All work runs in threads
}
